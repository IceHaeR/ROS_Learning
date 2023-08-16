
/******************************************************************
			2023-cometh-uavs_collaboration
*******************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <vector>
#include <thread>



// Structure for individual UAV data and operations
struct UAV {
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::TwistStamped velocity_cmd;
    geometry_msgs::PoseStamped target_pose;
    bool takeoff_flag = false;
    bool straight_flag = false;
    bool turn_flag = false;
    
    ros::Subscriber state_sub;
    ros::Subscriber pose_sub;
    ros::Publisher local_pos_pub;
    ros::Publisher velocity_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    
    void initialize(ros::NodeHandle &nh, int id);
};

void UAV::initialize(ros::NodeHandle &nh, int id) {
    std::string uav_name = "/iris_" + std::to_string(id);
    
    state_sub = nh.subscribe<mavros_msgs::State>(uav_name + "/mavros/state", 10, [this](const mavros_msgs::State::ConstPtr& msg){
        this->current_state = *msg;
    });

    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/local_position/pose", 10, [this](const geometry_msgs::PoseStamped::ConstPtr& msg){
        this->current_pose = *msg;
    });

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/mavros/setpoint_position/local", 10);
    velocity_pub = nh.advertise<geometry_msgs::TwistStamped>(uav_name + "/mavros/setpoint_velocity/cmd_vel", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_name + "/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_name + "/mavros/set_mode");
}

class FlightController {
	public:
	    FlightController(int uav_count, double flight_duration, int leader_id);
	    void runForUAV(UAV &uav, int uav_id);
	    void run();
	    bool takeoff(UAV &uav);
	    bool straight(UAV &uav, double duration);
	    bool turn(UAV &uav);
	    bool rtl(UAV &uav);
	    bool connectToFC(UAV &uav);
	    std::tuple<double, double, double> uav_gesture(UAV &uav); 

	private:
	    std::vector<UAV> uavs;
	    mavros_msgs::SetMode flight_set_mode;
	    mavros_msgs::CommandBool arm_cmd;
	    
	    UAV* leaderUAV;	//leader指针
	    
	    ros::Time time_flag;
	    ros::NodeHandle nh;
	    ros::Rate rate;
	    
	    double distance;
	    double roll;
	    double pitch;
	    double yaw;
	    double start_yaw;
	    double flight_time;
};

FlightController::FlightController(int uav_count, double flight_duration, int leader_id) : rate(20.0), flight_time(flight_duration) {   
    uavs.resize(uav_count);
    for(int i = 0; i < uav_count; i++) {
        uavs[i].initialize(nh, i);
        
        //为每台无人机设置不同的初始位置
        uavs[i].current_pose.pose.position.x = 0 + i * 0.5;
        uavs[i].current_pose.pose.position.y = 0;
        uavs[i].current_pose.pose.position.z = 0;
    }
    
    leaderUAV = &uavs[leader_id];
}

bool FlightController::takeoff(UAV &uav) {
    //起始位置设置
    uav.target_pose.pose.position.x = uav.current_pose.pose.position.x;
    uav.target_pose.pose.position.y = uav.current_pose.pose.position.y;
    uav.target_pose.pose.position.z = 3;
    
    //预发布期望位置信息，给无人机目标点
    for(int i = 100; ros::ok() && i > 0; --i){
        uav.local_pos_pub.publish(uav.target_pose);
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
        if(uav.current_state.mode != "OFFBOARD" && (ros::Time::now() - time_flag > ros::Duration(5.0))){
            if( uav.set_mode_client.call(flight_set_mode) && flight_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            time_flag = ros::Time::now();
        } else {
            if( !uav.current_state.armed && (ros::Time::now() - time_flag > ros::Duration(5.0))){
                if(uav.arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                time_flag = ros::Time::now();
            }
        }
        
        //判断无人机是否已经起飞至指定位置
        distance = sqrt(pow(uav.current_pose.pose.position.x - uav.target_pose.pose.position.x, 2) +
                       pow(uav.current_pose.pose.position.y - uav.target_pose.pose.position.y, 2) +
                       pow(uav.current_pose.pose.position.z - uav.target_pose.pose.position.z, 2));
        if(distance < 0.1){
            ROS_INFO("Takeoff successfully!");
            break;
        }
        //循环发布期望位置信息，20Hz频率
        uav.local_pos_pub.publish(uav.target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    return true;
}


bool FlightController::straight(UAV &uav, double flight_time) {
    //直行参数设置
    uav.velocity_cmd.twist.linear.x = 0.5 * cos(yaw);
    uav.velocity_cmd.twist.linear.y = 0.5 * sin(yaw);
    uav.velocity_cmd.twist.linear.z = 0;
    
    //获取时间戳
    time_flag = ros::Time::now();
    
    ROS_INFO("Switch to straight flight mode.");
    
    while(ros::ok()) {
        if(ros::Time::now() - time_flag > ros::Duration(flight_time)) {
            //姿态获取
            std::tie(roll, pitch, yaw) = uav_gesture(uav);
            start_yaw = yaw;
            break;
        }
        uav.velocity_pub.publish(uav.velocity_cmd);
        ros::spinOnce();
        rate.sleep();
    }
    
    return true;
}


bool FlightController::turn(UAV &uav) {
    //获取时间戳
    time_flag = ros::Time::now();
    ROS_INFO("Switch to turn mode.");
    
    while(ros::ok()) {
        //判断无人机转弯时姿态
        std::tie(roll, pitch, yaw) = uav_gesture(uav);
        if(abs(yaw - start_yaw) >= M_PI / 2){
            uav.velocity_cmd.twist.angular.z = 0;
            if(straight(uav, flight_time)){
                break;
            } else {
                ROS_INFO("Straight flight II errors");
            }
        } else {
            uav.velocity_cmd.twist.linear.x = 1 * cos(yaw);
            uav.velocity_cmd.twist.linear.y = 1 * sin(yaw);
            uav.velocity_cmd.twist.linear.z = 0;
            uav.velocity_cmd.twist.angular.z = 0.4;
        } 
        uav.velocity_pub.publish(uav.velocity_cmd);
        ros::spinOnce();
        rate.sleep();
    }
    
    return true;
}


bool FlightController::rtl(UAV &uav) {
    //获取时间戳
    time_flag = ros::Time::now();
    //设置客户端请求无人机进入“AUTO.RTL”模式
    flight_set_mode.request.custom_mode = "AUTO.RTL";
    
    while(ros::ok()) {
        if(uav.current_state.mode != "AUTO.RTL" && ros::Time::now() - time_flag > ros::Duration(5.0)) {
            if(uav.set_mode_client.call(flight_set_mode) && flight_set_mode.response.mode_sent){
                ROS_INFO("AUTO.RTL enabled");
                break;
            } 
        }
        uav.velocity_pub.publish(uav.velocity_cmd);
        ros::spinOnce();
        rate.sleep();
    }
    
    return true;
}


bool FlightController::connectToFC(UAV &uav) {
    //等待飞控连接
    while(ros::ok() && !uav.current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    
    if(uav.current_state.connected) {
        ROS_INFO("Successfully connected to the flight controller");
        return true;
    } else {
        ROS_INFO("Failed to connect to the flight controller");
        return false;
    }
}


std::tuple<double, double, double> FlightController::uav_gesture(UAV &uav) {
    tf::Quaternion q(
        uav.current_pose.pose.orientation.x, 
        uav.current_pose.pose.orientation.y, 
        uav.current_pose.pose.orientation.z, 
        uav.current_pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    return std::make_tuple(roll, pitch, yaw);  // 使用tuple返回三个值
}


void FlightController::runForUAV(UAV &uav, int uav_id) {
    if (!connectToFC(uav)) return;

    if (!uav.takeoff_flag) {
        takeoff(uav);
        uav.takeoff_flag = true;
    }

    while(ros::ok()) {
        if (&uav == leaderUAV) {
            if (!uav.straight_flag) {
                straight(uav, flight_time);
                uav.straight_flag = true;
            }

            if (!uav.turn_flag) {
                turn(uav);
                uav.turn_flag = true;
            }

            rtl(uav);
            break;
        } else {
            // 根据uav_id决定偏移量
            double offset_x = 0.0, offset_y = 0.0;
            switch(uav_id) {
                case 0: offset_x = 0.5; break;       // 第一架无人机向右移动
                case 1: offset_x = -0.5; break;      // 第二架无人机向左移动
                case 2: offset_x = 1.0; break;       // 第三架无人机向前移动
                case 3: offset_x = -1.0; break;       // 第四架无人机向前移动
                case 4: offset_x = 2.0; break;       // 第五架无人机向前移动
            }

            uav.target_pose.pose.position.x = leaderUAV->current_pose.pose.position.x + offset_x;
            uav.target_pose.pose.position.y = leaderUAV->current_pose.pose.position.y;
            uav.target_pose.pose.orientation = leaderUAV->current_pose.pose.orientation;
            uav.local_pos_pub.publish(uav.target_pose);
            ros::spinOnce();
            rate.sleep();
        }
    }
}



void FlightController::run() {
    std::vector<std::thread> threads;
    int uav_id = 0;
    for (UAV &uav : uavs) {
        threads.push_back(std::thread(&FlightController::runForUAV, this, std::ref(uav), uav_id));
        uav_id++;
    }

    for (auto &t : threads) {
        t.join();
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_tread_flight_node");
    
    if(argc != 4) {
    	ROS_ERROR("Usage: rosrun uavs_collaboration multi_tread_flight_node <uav_num> <flight_time>");
    	return -1;
    }
    
    int uav_count = std::stoi(argv[1]);
    double flight_duration = std::stod(argv[2]);
    int leader_id = std::stoi(argv[3]);
    
    FlightController fc(uav_count, flight_duration, leader_id);
    fc.run();
    return 0;
}
