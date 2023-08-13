#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>


class FollowController {
	public:
		FollowController();
		void run();
	
	private:
		void state_cb_B(const mavros_msgs::State::ConstPtr& msg);
		void pose_cb_A(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void pose_cb_B(const geometry_msgs::PoseStamped::ConstPtr& msg);
		bool connectToFC_B();
		bool takeoff_B();
		bool follow();
		bool land_B();
		
		mavros_msgs::State current_state_B;
		geometry_msgs::PoseStamped current_pose_A;
	    geometry_msgs::PoseStamped current_pose_B;
	    geometry_msgs::PoseStamped target_pose_B;
        mavros_msgs::SetMode flight_set_mode_B;
        mavros_msgs::CommandBool arm_cmd_B;
        
        ros::NodeHandle nh;
		ros::Subscriber state_sub_B;
		ros::Subscriber pose_sub_A;
		ros::Subscriber pose_sub_B;
		ros::Publisher local_pos_pub_B;
		ros::ServiceClient arming_client_B;
		ros::ServiceClient set_mode_client_B;
		ros::Rate rate;
		ros::Time time_flag;
		
		bool takeoff_flag_A;
		bool takeoff_flag_B;
		bool land_flag_A;
};

FollowController::FollowController() 
	: nh("~"),
	  rate(20.0),
	  takeoff_flag_A(false),
	  takeoff_flag_B(false),
	  land_flag_A(false)
{
	//创建订阅者，订阅话题/iris_1/mavros/state
	state_sub_B = nh.subscribe<mavros_msgs::State>("/iris_1/mavros/state", 10, &FollowController::state_cb_B, this);
	//创建订阅者，订阅主机A无人机节点位置信息
	pose_sub_A = nh.subscribe<geometry_msgs::PoseStamped>("/iris_0/mavros/local_position/pose", 10, &FollowController::pose_cb_A, this);
	//创建订阅者，订阅从机B无人机节点位置信息
	pose_sub_B = nh.subscribe<geometry_msgs::PoseStamped>("/iris_1/mavros/local_position/pose", 10, &FollowController::pose_cb_B, this);
	
	//创建发布者，发布从机B无人机目标位置信息
	local_pos_pub_B = nh.advertise<geometry_msgs::PoseStamped>("/iris_1/mavros/setpoint_position/local", 10);
	
	//创建一个客户端，请求px4无人机解锁
	arming_client_B = nh.serviceClient<mavros_msgs::CommandBool>("/iris_1/mavros/cmd/arming");
	//创建一个客户端，请求进入offboard模式
	set_mode_client_B = nh.serviceClient<mavros_msgs::SetMode>("/iris_1/mavros/set_mode");
}

//回调函数
void FollowController::state_cb_B(const mavros_msgs::State::ConstPtr& msg){
	current_state_B = *msg;
}

void FollowController::pose_cb_A(const geometry_msgs::PoseStamped::ConstPtr& msg){
	current_pose_A = *msg;
	target_pose_B.pose.orientation = current_pose_A.pose.orientation;
}

void FollowController::pose_cb_B(const geometry_msgs::PoseStamped::ConstPtr& msg){
	current_pose_B = *msg;
}

//uav飞行阶段
bool FollowController::connectToFC_B() {
	//等待飞控连接
	while(ros::ok() && !current_state_B.connected) {
		ros::spinOnce();
		rate.sleep();
	}
	
	if(current_state_B.connected) {
		ROS_INFO("Successfully connected to the flight controller");
		return true;
	} else {
		ROS_INFO("Failed to connect to the flight controller");
		return false;
	}
}

bool FollowController::takeoff_B() {
	//起始位置设置
	target_pose_B.pose.position.x = 3;
    target_pose_B.pose.position.y = 0;
    target_pose_B.pose.position.z = 3;
    //预发布期望位置信息，给无人机目标点
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub_B.publish(target_pose_B);
        ros::spinOnce();
        rate.sleep();
    }
    
    //设置客户端请求无人机进入“OFFBOARD”模式
    flight_set_mode_B.request.custom_mode = "OFFBOARD";
   
    //设置客户请无人机解锁
    arm_cmd_B.request.value = true;
 
    //获取时间戳
    time_flag = ros::Time::now();
    
	while(ros::ok()){
		//if语句循环请求进入OFFBOARD模式，进入以后则会进入else语句请求对无人机解锁。
		if( current_state_B.mode != "OFFBOARD" &&
			(ros::Time::now() - time_flag > ros::Duration(5.0))){
			if( set_mode_client_B.call(flight_set_mode_B) &&
				flight_set_mode_B.response.mode_sent){
				ROS_INFO("Offboard enabled");
			}
			time_flag = ros::Time::now();
		} else {
			if( !current_state_B.armed &&
				(ros::Time::now() - time_flag > ros::Duration(5.0))){
				if( arming_client_B.call(arm_cmd_B) &&
				    arm_cmd_B.response.success){
				    ROS_INFO("Vehicle armed");
				}
				time_flag = ros::Time::now();
			}
		}
				       
		//判断无人机是否已经起飞至指定位置
		double distance = sqrt(pow(current_pose_B.pose.position.x - target_pose_B.pose.position.x, 2) +
			       pow(current_pose_B.pose.position.y - target_pose_B.pose.position.y, 2) +
			       pow(current_pose_B.pose.position.z - target_pose_B.pose.position.z, 2));
		if(distance < 0.1){
			ROS_INFO("Takeoff successfully!");
			break;
		}
		
		//循环发布期望位置信息，20Hz频率
		local_pos_pub_B.publish(target_pose_B);
		ros::spinOnce();
		rate.sleep();
	}
	
	return true;
}

bool FollowController::follow() {
    // 等待获取主机A无人机位置信息
    while (ros::ok() && current_pose_A.header.stamp.isZero()) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Successfully obtained host A UAV information");

    while (ros::ok()) {
        target_pose_B.header.stamp = ros::Time::now();

        // 判断主机是否起飞
        if (!takeoff_flag_A && current_pose_A.pose.position.z >= 2.5) {
            takeoff_flag_A = true;
            ROS_INFO("Follow mode");
        }

        if (!takeoff_flag_A) {
            local_pos_pub_B.publish(target_pose_B);
        } else {
            // 根据主机A计算从机B的目标位置
            if (!land_flag_A && current_pose_A.pose.position.z < 0.1) {
                ros::Duration(5.0).sleep();
                if (current_pose_A.pose.position.z < 0.1) {
                    land_flag_A = true;
                }
            } else if (!land_flag_A) {
                target_pose_B.pose.position.x = current_pose_A.pose.position.x + 1.0;
                target_pose_B.pose.position.y = current_pose_A.pose.position.y;
                target_pose_B.pose.position.z = current_pose_A.pose.position.z + 1.0;
            }

            if (land_flag_A) {
                if (current_pose_B.pose.position.z < 0.1) {
                    ROS_INFO("Complete follow mission");
                    break;
                } else {
                    target_pose_B.pose.position.x = current_pose_A.pose.position.x + 2.0;
                    target_pose_B.pose.position.y = current_pose_A.pose.position.y;
                    target_pose_B.pose.position.z = 0.0;
                }
            }

            local_pos_pub_B.publish(target_pose_B);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return true;
}


void FollowController::run() {
	if(!connectToFC_B()) return;
	
	while(ros::ok()) {
		if(!takeoff_flag_B) {
			if(takeoff_B()) {
				takeoff_flag_B = true;
			}
		} else {
			if(follow()) {
				break;
			}
		}
		ros::spinOnce();
		rate.sleep();
	}
}

int main(int argc, char **argv) {
	//初始化节点
	ros::init(argc, argv, "follow_node");
	FollowController follow_controller;
	follow_controller.run();
	
	return 0;
}














