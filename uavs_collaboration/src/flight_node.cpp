/******************************************************************
			2023-cometh-uavs_collaboration
*******************************************************************/






//相关头文件包含
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <cmath>


class FlightController {
	public:
		FlightController();
		void run();
	
	private:
		void state_cb(const mavros_msgs::State::ConstPtr& msg);
		void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
		double uav_gesture(double x, double y, double z, double w);
		bool connectToFC();
		bool takeoff();
		bool straight(double flight_time);
		bool turn();
		bool rtl();
		
		mavros_msgs::State current_state;
		geometry_msgs::PoseStamped current_pose;
	    geometry_msgs::PoseStamped target_pose;
        geometry_msgs::TwistStamped velocity_cmd;
        mavros_msgs::SetMode flight_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        
        ros::NodeHandle nh;
		ros::Subscriber state_sub;
		ros::Subscriber pose_sub;
		ros::Publisher local_pos_pub;
		ros::Publisher velocity_pub;
		ros::ServiceClient arming_client;
		ros::ServiceClient set_mode_client;
		ros::Rate rate;
		ros::Time time_flag;
		
		bool takeoff_flag;
		bool straight_flag;
		bool turn_flag;
		double distance;
		double roll;
		double pitch;
		double yaw;
		double start_yaw;

};


FlightController::FlightController()
	: nh("~"),
	  rate(20.0),
	  takeoff_flag(false),
	  straight_flag(false),
	  turn_flag(false),
	  roll(0),
	  pitch(0),
	  start_yaw(0)
{
    //创建订阅者，订阅话题/iris_0/mavros/state
	state_sub = nh.subscribe<mavros_msgs::State>
		("/iris_0/mavros/state", 10, &FlightController::state_cb, this);
    //创建订阅者，订阅话题/iris_0/mavros/local_position/pose
	pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
		("/iris_0/mavros/local_position/pose", 10, &FlightController::pose_cb, this);
	
	//创建一个发布者，发布"/iris_0/mavros/setpoint_position/local"话题
	local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
		("/iris_0/mavros/setpoint_position/local", 10);
	//创建一个发布者，发布"/iris_0/mavros/setpoint_velocity/cmd_vel"话题
	velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
		("/iris_0/mavros/setpoint_velocity/cmd_vel", 10);
	
	//创建一个客户端，该客户端用来请求PX4无人机的解锁 
	arming_client = nh.serviceClient<mavros_msgs::CommandBool>
		("/iris_0/mavros/cmd/arming");
	//创建一个客户端，该客户端用来请求进入offboard模式
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
		("/iris_0/mavros/set_mode");			
}	  

//回调函数
void FlightController::state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

void  FlightController::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	current_pose = *msg;
} 

//姿态转换
double FlightController::uav_gesture(double x, double y, double z, double w) {
	tf::Quaternion q(
		x, y, z, w
	);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
	return roll, pitch, yaw;
}

//uav飞行阶段
bool FlightController::connectToFC() {
	//等待飞控连接
	while(ros::ok() && !current_state.connected) {
		ros::spinOnce();
		rate.sleep();
	}
	
	if(current_state.connected) {
		ROS_INFO("Successfully connected to the flight controller");
		return true;
	} else {
		ROS_INFO("Failed to connect to the flight controller");
		return false;
	}
}

bool FlightController::takeoff() {
	//起始位置设置
	target_pose.pose.position.x = 0;
    target_pose.pose.position.y = 0;
    target_pose.pose.position.z = 5;
    //预发布期望位置信息，给无人机目标点
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    //设置客户端请求无人机进入“OFFBOARD”模式
    flight_set_mode.request.custom_mode = "OFFBOARD";
   
    //设置客户请无人机解锁
    arm_cmd.request.value = true;
 
    //获取时间戳
    time_flag = ros::Time::now();
    
	while(ros::ok()){
		//if语句循环请求进入OFFBOARD模式，进入以后则会进入else语句请求对无人机解锁。
		if( current_state.mode != "OFFBOARD" &&
			(ros::Time::now() - time_flag > ros::Duration(5.0))){
			if( set_mode_client.call(flight_set_mode) &&
				flight_set_mode.response.mode_sent){
				ROS_INFO("Offboard enabled");
			}
			time_flag = ros::Time::now();
		} else {
			if( !current_state.armed &&
				(ros::Time::now() - time_flag > ros::Duration(5.0))){
				if( arming_client.call(arm_cmd) &&
				    arm_cmd.response.success){
				    ROS_INFO("Vehicle armed");
				}
				time_flag = ros::Time::now();
			}
		}
				       
		//判断无人机是否已经起飞至指定位置
		distance = sqrt(pow(current_pose.pose.position.x - target_pose.pose.position.x, 2) +
			       pow(current_pose.pose.position.y - target_pose.pose.position.y, 2) +
			       pow(current_pose.pose.position.z - target_pose.pose.position.z, 2));
		if(distance < 0.1){
			ROS_INFO("Takeoff successfully!");
			ROS_INFO("Switch to straight flight mode.");
			break;
		}
		//循环发布期望位置信息，20Hz频率
		local_pos_pub.publish(target_pose);
		ros::spinOnce();
		rate.sleep();
	}
	
	return true;
}

bool FlightController::straight(double flight_time) {
	//直行参数设置
	velocity_cmd.twist.linear.x = 0.5 * cos(yaw);
    velocity_cmd.twist.linear.y = 0.5 * sin(yaw);
    velocity_cmd.twist.linear.z = 0;
    
    //获取时间戳
    time_flag = ros::Time::now();
    
    while(ros::ok()) {
    	if(ros::Time::now() - time_flag > ros::Duration(flight_time)) {
    		//姿态获取
                roll, pitch, yaw = uav_gesture(current_pose.pose.orientation.x,
											current_pose.pose.orientation.y,
											current_pose.pose.orientation.z,
											current_pose.pose.orientation.w);
				start_yaw = yaw;
				ROS_INFO("Switch to turn mode.");
				break;
    	}
    	velocity_pub.publish(velocity_cmd);
    	ros::spinOnce();
    	rate.sleep();
    }
    
    return true;
}
 
bool FlightController::turn() {
    //获取时间戳
    time_flag = ros::Time::now();
    while(ros::ok()) {
		//判断无人机转弯时姿态
		//四元数和旋转矩阵
		roll, pitch, yaw = uav_gesture(current_pose.pose.orientation.x,
										current_pose.pose.orientation.y,
										current_pose.pose.orientation.z,
										current_pose.pose.orientation.w);
		if(abs(yaw - start_yaw) >= M_PI / 2){
			ROS_INFO("Switch to straight mode.");
			velocity_cmd.twist.angular.z = 0;
			if(FlightController::straight(30.0)){
				break;
			} else {
				ROS_INFO("Straight fligh II errors");
			}
		} else {
			velocity_cmd.twist.linear.x = 1 * cos(yaw);
			velocity_cmd.twist.linear.y = 1 * sin(yaw);
			velocity_cmd.twist.linear.z = 0;
			velocity_cmd.twist.angular.z = 0.4;
		} 
		velocity_pub.publish(velocity_cmd);
		ros::spinOnce();
		rate.sleep();
	}
	
	return true;
}
 
bool FlightController::rtl() {
	//获取时间戳
    time_flag = ros::Time::now();
    //设置客户端请求无人机进入“OFFBOARD”模式
    flight_set_mode.request.custom_mode = "AUTO.RTL";
    //自动返回
    while(ros::ok()) {
    	if(current_state.mode != "AUTO.RTL" && ros::Time::now() - time_flag > ros::Duration(5.0)) {
			if(set_mode_client.call(flight_set_mode) && flight_set_mode.response.mode_sent){
	    		ROS_INFO("AUTO.RTL enabled");
	    		break;
			} 
		}
        velocity_pub.publish(velocity_cmd);
    	ros::spinOnce();
        rate.sleep();
    }
    
    return true;
}

void FlightController::run() {
	if(!connectToFC()) return;
	
	while(ros::ok()) {
		if(!takeoff_flag) {
			if(takeoff()) {
				takeoff_flag = true;
			}
		} else if(!straight_flag) {
			if(straight(40.0)) {
				straight_flag = true;
			}
		} else if(!turn_flag) {
			if(turn()) {
				turn_flag = true;
			}
		} else {
			if(rtl()) {
				break;
			}
		}
		ros::spinOnce();
		rate.sleep();
	}
}
 
int main(int argc, char ** argv) {
	//初始化节点
	ros::init(argc, argv, "flight_node");
	FlightController flight_controller;
	flight_controller.run();
	
	return 0;
}
 
 
 
 
 
 
 
 












