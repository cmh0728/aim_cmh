#include "ros/ros.h"
#include "std_msgs/Int32.h"

void callback(const std_msgs::Int32::ConstPtr& msg) {
    ROS_INFO("Callback received: %d", msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pub_sub_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Int32>("pub_topic", 10);
    ros::Subscriber sub = nh.subscribe("pub_topic", 10, callback);

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
