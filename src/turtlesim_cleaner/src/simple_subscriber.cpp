#include "ros/ros.h"
#include "std_msgs/Int32.h"

void callback(const std_msgs::Int32::ConstPtr& msg) {
    ROS_INFO("Received: %d", msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("topic_name", 10, callback);
    ros::spin();

    return 0;
}
