/******************************************************************
			2023-cometh-uavs_collaboration
*******************************************************************/


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>


geometry_msgs::PoseStamped current_pose_A; //主机A位置
geometry_msgs::PoseStamped target_pose_B; //从机B位置
mavros_msgs::State current_state_B;

//回调函数
void state_cb_B(const mavros_msgs::State::ConstPtr& msg){
	current_state_B = *msg;
}

void pose_cb_A(const geometry_msgs::PoseStamped::ConstPtr& msg){
	current_pose_A = *msg;
	target_pose_B.pose.orientation = current_pose_A.pose.orientation;
}

int main(int argc, char **argv)
{
	//初始化节点
	ros::init(argc, argv, "follow_node");
	//创建节点句柄
	ros::NodeHandle nh;
	
	//创建订阅者，订阅话题/iris_1/mavros/state
	ros::Subscriber state_sub_B = nh.subscribe<mavros_msgs::State>("/iris_1/mavros/state", 10, state_cb_B);
	//创建订阅者，订阅主机A无人机节点位置信息
	ros::Subscriber pose_sub_A = nh.subscribe<geometry_msgs::PoseStamped>("/iris_0/mavros/local_position/pose", 10, pose_cb_A);
	
	//创建发布者，发布从机B无人机目标位置信息
	ros::Publisher local_pos_pub_B = nh.advertise<geometry_msgs::PoseStamped>("/iris_1/mavros/setpoint_position/local", 10);
	
	//创建一个客户端，请求px4无人机解锁
	ros::ServiceClient arming_client_B = nh.serviceClient<mavros_msgs::CommandBool>("/iris_1/mavros/cmd/arming");
	//创建一个客户端，请求进入offboard模式
	ros::ServiceClient set_mode_client_B = nh.serviceClient<mavros_msgs::SetMode>("/iris_1/mavros/set_mode");
	
	ros::Rate rate(20.0);
	
	//等待飞控连接
	while(ros::ok() && !current_state_B.connected) {
		ros::spinOnce();
		rate.sleep();
	}
	
	if(current_state_B.connected){
		ROS_INFO("Successfully connected to the flight controllor");	
	} else {
		ROS_INFO("Failed to connect to the flight controllor");
	}
	
	
	
	//从机无人机B初始位置
	target_pose_B.pose.position.x = 0.0;
	target_pose_B.pose.position.y = 3.0;
	target_pose_B.pose.position.z = 3.0;
	
	//预发布从机B位置信息，使其启动
	for(int i = 100; ros::ok() && i > 0; --i){
		local_pos_pub_B.publish(target_pose_B);
		ros::spinOnce();
		rate.sleep();
	}
	
	//设置客户端请求无人机进入“offboard”模式
	mavros_msgs::SetMode flight_set_mode_B;
	flight_set_mode_B.request.custom_mode = "OFFBOARD";
	
	//设置客户端请求无人机解锁
	mavros_msgs::CommandBool arm_cmd_B;
	arm_cmd_B.request.value = true;
	
	
	//等待获取主机A无人机位置信息
	while(ros::ok() && current_pose_A.header.stamp.isZero()){
		ros::spinOnce;
		rate.sleep();
	}
	
	ROS_INFO("Successfully obtained host A UAV information");
	
	//获取时间戳
	ros::Time time_flag = ros::Time::now();
	
	//发布频率为20Hz
	while(ros::ok()){
		//从机B无人机起飞
		if(current_state_B.mode != "OFFBOARD" &&
		(ros::Time::now() - time_flag > ros::Duration(5.0))) {
			if(set_mode_client_B.call(flight_set_mode_B) && 
			flight_set_mode_B.response.mode_sent) { 
				ROS_INFO("offboard enabled");
			}
			time_flag = ros::Time::now();
		} else {
			if(!current_state_B.armed && 
			(ros::Time::now() - time_flag > ros::Duration(5.0))) {
				if(arming_client_B.call(arm_cmd_B) && arm_cmd_B.response.success) {
					ROS_INFO("Vehicle armed");
				}
				time_flag = ros::Time::now();
			}
		}
	
		//根据主机A计算从机B的目标位置
		target_pose_B.pose.position.x = current_pose_A.pose.position.x;
		target_pose_B.pose.position.y = current_pose_A.pose.position.y;
		target_pose_B.pose.position.z = current_pose_A.pose.position.z + 2.0;
		
		
		//发布从机B的目标位置信息
		target_pose_B.header.stamp = ros::Time::now();
		local_pos_pub_B.publish(target_pose_B);
		
		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}
















