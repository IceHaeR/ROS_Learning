#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

class tfBroadcasterController {
    public:
        tfBroadcasterController();
        void run();
    private:
        void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

        geometry_msgs::PoseStamped current_pose;
        tf::TransformBroadcaster broadcaster;

        ros::NodeHandle nh;
        ros::Subscriber pose_sub;
        ros::Rate rate;
        ros::Time time_flag;
};

tfBroadcasterController::tfBroadcasterController():nh("~"), rate(10.0) {
    //subscribe topic "/iris_0/mavros/local_position/pose"
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/iris_0/mavros/local_position/pose", 10, &tfBroadcasterController::pose_cb, this);
}

void tfBroadcasterController::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "iris_0"));
}

void tfBroadcasterController::run() {
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "iris0_tf_broadcaster");

    tfBroadcasterController broadcaster_controller;
    broadcaster_controller.run();

    return 0;
}
