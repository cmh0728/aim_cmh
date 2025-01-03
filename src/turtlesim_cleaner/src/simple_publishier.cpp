#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Int32>("topic_name", 10);

    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok()) {
        std_msgs::Int32 msg;
        msg.data = count;
        pub.publish(msg);

        ROS_INFO("Published: %d", msg.data);
        count++;
        loop_rate.sleep();
    }
    return 0;
}
