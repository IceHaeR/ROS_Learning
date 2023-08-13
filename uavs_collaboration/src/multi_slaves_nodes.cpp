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

class MultislaveController {
public:
    MultislaveController();
    void run();
    void switchFormation(const std::string& formation_name);
    double getYawFromQuaternion(const geometry_msgs::Quaternion& q); //获取leader偏转角

private:
	void state_cb_leader(const mavros_msgs::State::ConstPtr& msg);
	void velocity_cb_leader(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void state_cb(const mavros_msgs::State::ConstPtr& msg, int idx);
    void pose_cb_leader(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool all_slaves_connected();
    void set_mode_and_arm(int idx);
    void update_target_positions();
    void set_slaves_to_rtl();

    geometry_msgs::PoseStamped current_pose_leader;
    geometry_msgs::TwistStamped current_velocity_leader;
    geometry_msgs::PoseStamped target_pose_slaves[5];
    mavros_msgs::State current_state_leader;
    mavros_msgs::State current_state_slaves[5];

    ros::NodeHandle nh;
    ros::Subscriber pose_sub_leader;
    ros::Subscriber state_sub_leader;
    ros::Subscriber velocity_sub_leader;
    ros::Subscriber state_sub_slaves[5];
    ros::Publisher local_pos_pub_slaves[5];
    ros::ServiceClient arming_client_slaves[5];
    ros::ServiceClient set_mode_client_slaves[5];
    ros::Rate rate;
    ros::Time time_flag[5];

    std::map<std::string, std::vector<std::vector<double>>> formations; // Predefined formations
	std::vector<std::vector<double>> current_formation_offsets; // Current formation's position offsets

};

MultislaveController::MultislaveController() : nh("~"), rate(20.0) {
    // Define predefined formations
    formations["line"] = {{-4.0, 0.0, 0.0}, {-6.0, 0.0, 0.0}, {-8.0, 0.0, 0.0}, {-2.0, 0.0, 0.0}, {-10.0, 0.0, 0.0}};
  

    // Initial formation
    switchFormation("line");

	//订阅与发布
    pose_sub_leader = nh.subscribe<geometry_msgs::PoseStamped>("/iris_0/mavros/local_position/pose", 10, &MultislaveController::pose_cb_leader, this);
    state_sub_leader = nh.subscribe<mavros_msgs::State>("/iris_0/mavros/state", 10, &MultislaveController::state_cb_leader, this);
    velocity_sub_leader = nh.subscribe<geometry_msgs::TwistStamped>("iris_0/mavros/local_position/velocity_local", 10, &MultislaveController::velocity_cb_leader, this);
    
    for(int i = 5; i <= 9; i++) {
        state_sub_slaves[i-5] = nh.subscribe<mavros_msgs::State>("/iris_" + std::to_string(i) + "/mavros/state", 10, boost::bind(&MultislaveController::state_cb, this, _1, i-5));
        local_pos_pub_slaves[i-5] = nh.advertise<geometry_msgs::PoseStamped>("/iris_" + std::to_string(i) + "/mavros/setpoint_position/local", 10);
        arming_client_slaves[i-5] = nh.serviceClient<mavros_msgs::CommandBool>("/iris_" + std::to_string(i) + "/mavros/cmd/arming");
        set_mode_client_slaves[i-5] = nh.serviceClient<mavros_msgs::SetMode>("/iris_" + std::to_string(i) + "/mavros/set_mode");
        time_flag[i-5] = ros::Time::now();
    }
}

void MultislaveController::switchFormation(const std::string& formation_name) {
    if (formations.find(formation_name) != formations.end()) {
        current_formation_offsets = formations[formation_name];
    } else {
        ROS_WARN("Formation name not found: %s", formation_name.c_str());
    }
}

//回调函数
void MultislaveController::state_cb_leader(const mavros_msgs::State::ConstPtr& msg) {
	current_state_leader = *msg;
}

void MultislaveController::state_cb(const mavros_msgs::State::ConstPtr& msg, int idx) {
    current_state_slaves[idx] = *msg;
}

void MultislaveController::pose_cb_leader(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_leader = *msg;
    for(int i = 0; i < 5; i++) {
        target_pose_slaves[i].pose.orientation = current_pose_leader.pose.orientation;
    }
}

void MultislaveController::velocity_cb_leader(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	current_velocity_leader = *msg;
}


bool MultislaveController::all_slaves_connected() {
    for(int i = 0; i < 5; i++) {
        if(!current_state_slaves[i].connected) {
            return false;
        }
    }
    return true;
}

void MultislaveController::set_mode_and_arm(int idx) {
    if (ros::Time::now() - time_flag[idx] < ros::Duration(5.0)) return;

    mavros_msgs::SetMode flight_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    flight_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    if(current_state_slaves[idx].mode != "OFFBOARD" && set_mode_client_slaves[idx].call(flight_set_mode) && flight_set_mode.response.mode_sent) {
        ROS_INFO("offboard enabled for slave %d", idx + 1);
    } else if(!current_state_slaves[idx].armed && arming_client_slaves[idx].call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed for slave %d", idx + 1);
    }
	
	local_pos_pub_slaves[idx].publish(target_pose_slaves[idx]);
    time_flag[idx] = ros::Time::now();
}

//或缺leader偏转角
double MultislaveController::getYawFromQuaternion(const geometry_msgs::Quaternion& q) {
	//四元数转欧拉角
	tf::Quaternion tfq(q.x, q.y, q.z, q.w);
	tf::Matrix3x3 m(tfq);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return yaw;
}

void MultislaveController::update_target_positions() {
	double yaw = getYawFromQuaternion(current_pose_leader.pose.orientation); //获取leader偏转角
	
	const double safe_time = 1.0;
	double predicted_x = current_pose_leader.pose.position.x + current_velocity_leader.twist.linear.x * safe_time;
	double predicted_y = current_pose_leader.pose.position.y + current_velocity_leader.twist.linear.y * safe_time;
	double predicted_z = current_pose_leader.pose.position.z + current_velocity_leader.twist.linear.z * safe_time;
	
    for(int i = 0; i < 5; i++) {
    	//结合二维旋转矩阵，使用yaw，使slaves与leader保持相对静止
    	double x_rotated = current_formation_offsets[i][0] * cos(yaw) - current_formation_offsets[i][1] * sin(yaw);
    	double y_rotated = current_formation_offsets[i][0] * sin(yaw) + current_formation_offsets[i][1] * cos(yaw);
        //更新目标位置
        target_pose_slaves[i].pose.position.x = predicted_x + x_rotated;
        target_pose_slaves[i].pose.position.y = predicted_y + y_rotated;
        target_pose_slaves[i].pose.position.z = predicted_z + current_formation_offsets[i][2];
        target_pose_slaves[i].header.stamp = ros::Time::now();
        local_pos_pub_slaves[i].publish(target_pose_slaves[i]);
    }
}

//设置slaves返航
void MultislaveController::set_slaves_to_rtl() {
	mavros_msgs::SetMode set_mode;
	set_mode.request.custom_mode = "AUTO.RTL";
	
	for(int i = 0; i < 5; i++) {
		if (set_mode_client_slaves[i].call(set_mode) && set_mode.response.mode_sent) {
			ROS_INFO("slave %d set to AUTO.RTL mode", i);
		} else {
			ROS_ERROR("Failed to set slave %d to AUTO.RTL mode", i);
		}
	}
}

void MultislaveController::run() {
    while (!all_slaves_connected() && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    // Set initial target positions
    for(int i = 0; i < 5; i++) {
        target_pose_slaves[i].pose.position.x = 0.0;
        target_pose_slaves[i].pose.position.y = 3.0 + 2*(i+1);
        target_pose_slaves[i].pose.position.z = 3.0;
    }

    for(int i = 100; ros::ok() && i > 0; --i) {
        for(int j = 0; j < 5; j++) {
            local_pos_pub_slaves[j].publish(target_pose_slaves[j]);
        }
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()) {
        for(int i = 0; i < 5; i++) {
            set_mode_and_arm(i);
        }
        if(current_pose_leader.pose.position.z > 2.0) {
        	
        	
        	update_target_positions();
        	if (current_state_leader.mode == "AUTO.RTL") {
		    	set_slaves_to_rtl();
		    	break;
        	}
        } else {
        	for(int i = 0; i < 5; i++) {
        		local_pos_pub_slaves[i].publish(target_pose_slaves[i]);
        	}
        }
        
        
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_formation_nodes");
    MultislaveController controller;

    // Example: Switching formations during flight
    //controller.switchFormation("triangle");
    controller.run();


    return 0;
}

