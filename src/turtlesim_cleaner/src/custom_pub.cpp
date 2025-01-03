#include "ros/ros.h"
#include "turtlesim_cleaner/MyCustom.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "custom_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<turtlesim_cleaner::MyCustom>("custom_topic", 10);
    ros::Rate rate(10);

    while (ros::ok()) {
        turtlesim_cleaner::MyCustom msg;
        msg.x = 1.23;
        msg.y = 4.56;
        msg.arr = {7.89, 10.11, 12.13};

        ROS_INFO("Publishing: x=%f, y=%f, arr[0]=%f", msg.x, msg.y, msg.arr[0]);
        pub.publish(msg);

        rate.sleep();
    }
    return 0;
}
