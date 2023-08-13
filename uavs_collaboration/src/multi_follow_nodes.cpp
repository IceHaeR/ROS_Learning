#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>

class MultiFollowerController {
public:
    MultiFollowerController();
    void run();

private:
    void state_cb(const mavros_msgs::State::ConstPtr& msg, int idx);
    void pose_cb_leader(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool all_followers_connected();
    void set_mode_and_arm(int idx);
    void update_target_positions();

    geometry_msgs::PoseStamped current_pose_leader;
    geometry_msgs::PoseStamped target_pose_followers[4];
    mavros_msgs::State current_state_followers[4];

    ros::NodeHandle nh;
    ros::Subscriber pose_sub_leader;
    ros::Subscriber state_sub_followers[4];
    ros::Publisher local_pos_pub_followers[4];
    ros::ServiceClient arming_client_followers[4];
    ros::ServiceClient set_mode_client_followers[4];
    ros::Rate rate;
    ros::Time time_flag[4];
};

MultiFollowerController::MultiFollowerController() : nh("~"), rate(20.0) {
    pose_sub_leader = nh.subscribe<geometry_msgs::PoseStamped>("/iris_0/mavros/local_position/pose", 10, &MultiFollowerController::pose_cb_leader, this);
    for(int i = 1; i <= 4; i++) {
        state_sub_followers[i-1] = nh.subscribe<mavros_msgs::State>("/iris_" + std::to_string(i) + "/mavros/state", 10, boost::bind(&MultiFollowerController::state_cb, this, _1, i-1));
        local_pos_pub_followers[i-1] = nh.advertise<geometry_msgs::PoseStamped>("/iris_" + std::to_string(i) + "/mavros/setpoint_position/local", 10);
        arming_client_followers[i-1] = nh.serviceClient<mavros_msgs::CommandBool>("/iris_" + std::to_string(i) + "/mavros/cmd/arming");
        set_mode_client_followers[i-1] = nh.serviceClient<mavros_msgs::SetMode>("/iris_" + std::to_string(i) + "/mavros/set_mode");
        time_flag[i-1] = ros::Time::now();
    }
}

void MultiFollowerController::state_cb(const mavros_msgs::State::ConstPtr& msg, int idx) {
    current_state_followers[idx] = *msg;
}

void MultiFollowerController::pose_cb_leader(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_leader = *msg;
    for(int i = 0; i < 4; i++) {
        target_pose_followers[i].pose.orientation = current_pose_leader.pose.orientation;
    }
}

bool MultiFollowerController::all_followers_connected() {
    for(int i = 0; i < 4; i++) {
        if(!current_state_followers[i].connected) {
            return false;
        }
    }
    return true;
}

void MultiFollowerController::set_mode_and_arm(int idx) {
    if (ros::Time::now() - time_flag[idx] < ros::Duration(5.0)) return;

    mavros_msgs::SetMode flight_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    flight_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    if(current_state_followers[idx].mode != "OFFBOARD" && set_mode_client_followers[idx].call(flight_set_mode) && flight_set_mode.response.mode_sent) {
        ROS_INFO("offboard enabled for follower %d", idx + 1);
    } else if(!current_state_followers[idx].armed && arming_client_followers[idx].call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed for follower %d", idx + 1);
    }

    time_flag[idx] = ros::Time::now();
}

void MultiFollowerController::update_target_positions() {
    for(int i = 0; i < 4; i++) {
        target_pose_followers[i].pose.position.x = current_pose_leader.pose.position.x + i * 2.0;
        target_pose_followers[i].pose.position.y = current_pose_leader.pose.position.y + i * 2.0;
        target_pose_followers[i].pose.position.z = current_pose_leader.pose.position.z + 2.0;
        target_pose_followers[i].header.stamp = ros::Time::now();
        local_pos_pub_followers[i].publish(target_pose_followers[i]);
    }
}

void MultiFollowerController::run() {
    // Wait until all followers are connected
    while(ros::ok() && !all_followers_connected()) {
        ros::spinOnce();
        rate.sleep();
    }

    // Publish target positions for initial 100 times
    for(int i = 0; i < 100 && ros::ok(); i++) {
        update_target_positions();
        ros::spinOnce();
        rate.sleep();
    }

    // Main loop
    while(ros::ok()) {
        for(int i = 0; i < 4; i++) {
            set_mode_and_arm(i);
        }
        update_target_positions();
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_follow_nodes");
    MultiFollowerController controller;
    controller.run();
    return 0;
}

