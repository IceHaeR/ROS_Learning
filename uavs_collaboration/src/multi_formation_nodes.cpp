#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <map>
#include <string>
#include <vector>
#include <tf/transform_datatypes.h>
#include <cmath>

class MultiFollowerController {
public:
    MultiFollowerController();
    void run();
    void switchFormation(const std::string& formation_name);
    double getYawFromQuaternion(const geometry_msgs::Quaternion& q); //获取leader偏转角

private:
	void state_cb_leader(const mavros_msgs::State::ConstPtr& msg);
	void velocity_cb_leader(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void state_cb(const mavros_msgs::State::ConstPtr& msg, int idx);
    void pose_cb_leader(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool all_followers_connected();
    void set_mode_and_arm(int idx);
    void update_target_positions();
    void set_followers_to_rtl();

    geometry_msgs::PoseStamped current_pose_leader;
    geometry_msgs::TwistStamped current_velocity_leader;
    geometry_msgs::PoseStamped target_pose_followers[4];
    mavros_msgs::State current_state_leader;
    mavros_msgs::State current_state_followers[4];

    ros::NodeHandle nh;
    ros::Subscriber pose_sub_leader;
    ros::Subscriber state_sub_leader;
    ros::Subscriber velocity_sub_leader;
    ros::Subscriber state_sub_followers[4];
    ros::Publisher local_pos_pub_followers[4];
    ros::ServiceClient arming_client_followers[4];
    ros::ServiceClient set_mode_client_followers[4];
    ros::Rate rate;
    ros::Time time_flag[4];

    std::map<std::string, std::vector<std::vector<double>>> formations; // Predefined formations
	std::vector<std::vector<double>> current_formation_offsets; // Current formation's position offsets
    //单架无人机标签
    bool drone_control_flag; 
    std::string selected_drone_;

};

MultiFollowerController::MultiFollowerController() : nh("~"), rate(20.0),  drone_control_flag(false){
    // Define predefined formations
    formations["line"] = {{4.0, 0.0, 0.0}, {6.0, 0.0, 0.0}, {8.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
    formations["triangle"] = {{3.0, 0.0, 0.0}, {3.0, 2.0, 0.0}, {3.0, 4.0, 0.0}, {1.5, 2.0, 0.0}};
    formations["square"] = {{-2.0, -2.0, 0.0}, {2.0, -2.0, 0.0}, {2.0, 2.0, 0.0}, {-2.0, 2.0, 0.0}};

    // Initial formation
    switchFormation("line");

	//订阅与发布
    pose_sub_leader = nh.subscribe<geometry_msgs::PoseStamped>("/iris_0/mavros/local_position/pose", 10, &MultiFollowerController::pose_cb_leader, this);
    state_sub_leader = nh.subscribe<mavros_msgs::State>("/iris_0/mavros/state", 10, &MultiFollowerController::state_cb_leader, this);
    velocity_sub_leader = nh.subscribe<geometry_msgs::TwistStamped>("iris_0/mavros/local_position/velocity_local", 10, &MultiFollowerController::velocity_cb_leader, this);
    
    for(int i = 1; i <= 4; i++) {
        state_sub_followers[i-1] = nh.subscribe<mavros_msgs::State>("/iris_" + std::to_string(i) + "/mavros/state", 10, boost::bind(&MultiFollowerController::state_cb, this, _1, i-1));
        local_pos_pub_followers[i-1] = nh.advertise<geometry_msgs::PoseStamped>("/iris_" + std::to_string(i) + "/mavros/setpoint_position/local", 10);
        arming_client_followers[i-1] = nh.serviceClient<mavros_msgs::CommandBool>("/iris_" + std::to_string(i) + "/mavros/cmd/arming");
        set_mode_client_followers[i-1] = nh.serviceClient<mavros_msgs::SetMode>("/iris_" + std::to_string(i) + "/mavros/set_mode");
        time_flag[i-1] = ros::Time::now();
    }
}

void MultiFollowerController::switchFormation(const std::string& formation_name) {
    if (formations.find(formation_name) != formations.end()) {
        current_formation_offsets = formations[formation_name];
    } else {
        ROS_WARN("Formation name not found: %s", formation_name.c_str());
    }
}

//回调函数
void MultiFollowerController::state_cb_leader(const mavros_msgs::State::ConstPtr& msg) {
	current_state_leader = *msg;
    //选中一架无人机单独操作
    if (msg->mode == "OFFBOARD" && !drone_control_flag) {
        drone_control_flag = true;

        //随机挑选一架无人机 iris_1-iris_4
        int drone_num = 1 + rand() % 4;
        selected_drone_ = "iris_" + std::to_string(drone_num);
        ROS_INFO("Selected drone: %s", selected_drone_.c_str());
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

void MultiFollowerController::velocity_cb_leader(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	current_velocity_leader = *msg;
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
	
	local_pos_pub_followers[idx].publish(target_pose_followers[idx]);
    time_flag[idx] = ros::Time::now();
}

//或缺leader偏转角
double MultiFollowerController::getYawFromQuaternion(const geometry_msgs::Quaternion& q) {
	//四元数转欧拉角
	tf::Quaternion tfq(q.x, q.y, q.z, q.w);
	tf::Matrix3x3 m(tfq);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return yaw;
}

void MultiFollowerController::update_target_positions() {
	double yaw = getYawFromQuaternion(current_pose_leader.pose.orientation); //获取leader偏转角
	
	const double safe_time = 1.0;
	double predicted_x = current_pose_leader.pose.position.x + current_velocity_leader.twist.linear.x * safe_time;
	double predicted_y = current_pose_leader.pose.position.y + current_velocity_leader.twist.linear.y * safe_time;
	double predicted_z = current_pose_leader.pose.position.z + current_velocity_leader.twist.linear.z * safe_time;
	
    for(int i = 0; i < 4; i++) {
        std::string drone_name = "iris_" + std::to_string(i+1);
        if(drone_control_flag && drone_name == selected_drone_) {
            //对选定无人机发送命令
            target_pose_followers[i].pose.position.x += 0.0001;
        } else {
            //结合二维旋转矩阵，使用yaw，使followers与leader保持相对静止
            double x_rotated = current_formation_offsets[i][0] * cos(yaw) - current_formation_offsets[i][1] * sin(yaw);
            double y_rotated = current_formation_offsets[i][0] * sin(yaw) + current_formation_offsets[i][1] * cos(yaw);
            //更新目标位置
            target_pose_followers[i].pose.position.x = predicted_x + x_rotated;
            target_pose_followers[i].pose.position.y = predicted_y + y_rotated;
            target_pose_followers[i].pose.position.z = predicted_z + current_formation_offsets[i][2];
            
        }
        target_pose_followers[i].header.stamp = ros::Time::now();
        local_pos_pub_followers[i].publish(target_pose_followers[i]);

    }
}

//设置followers返航
void MultiFollowerController::set_followers_to_rtl() {
	mavros_msgs::SetMode set_mode;
	set_mode.request.custom_mode = "AUTO.RTL";
	
	for(int i = 0; i < 4; i++) {
		if (set_mode_client_followers[i].call(set_mode) && set_mode.response.mode_sent) {
			ROS_INFO("Follower %d set to AUTO.RTL mode", i);
		} else {
			ROS_ERROR("Failed to set follower %d to AUTO.RTL mode", i);
		}
	}
}

void MultiFollowerController::run() {
    while (!all_followers_connected() && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    // Set initial target positions
    for(int i = 0; i < 4; i++) {
        target_pose_followers[i].pose.position.x = 0.0;
        target_pose_followers[i].pose.position.y = 3.0 - 2*(i+1);
        target_pose_followers[i].pose.position.z = 3.0;
    }

    for(int i = 100; ros::ok() && i > 0; --i) {
        for(int j = 0; j < 4; j++) {
            local_pos_pub_followers[j].publish(target_pose_followers[j]);
        }
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()) {
        for(int i = 0; i < 4; i++) {
            set_mode_and_arm(i);
        }
        if(current_pose_leader.pose.position.z > 2.0) {
        	/*if(current_pose_leader.pose.position.x > 10.0) {
        		switchFormation("line");
        	}*/
        	
        	update_target_positions();
        	if (current_state_leader.mode == "AUTO.RTL") {
		    	set_followers_to_rtl();
		    	break;
        	}
        } else {
        	for(int i = 0; i < 4; i++) {
        		local_pos_pub_followers[i].publish(target_pose_followers[i]);
        	}
        }
        
        
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_formation_nodes");
    MultiFollowerController controller;

    // Example: Switching formations during flight
    //controller.switchFormation("triangle");
    controller.run();


    return 0;
}

