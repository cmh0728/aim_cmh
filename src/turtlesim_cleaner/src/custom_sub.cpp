#include "ros/ros.h"
#include "turtlesim_cleaner/MyCustom.h"

void callback(const turtlesim_cleaner::MyCustom::ConstPtr& msg) {
    ROS_INFO("Received: x=%f, y=%f, arr[0]=%f", msg->x, msg->y, msg->arr[0]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "custom_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("custom_topic", 5, callback);
    ros::spin();

    return 0;
}
