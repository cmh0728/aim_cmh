#include "ros/ros.h"
#include "std_msgs/Int32.h"

void callback(const std_msgs::Int32::ConstPtr& msg) {
    ROS_INFO("Received: %d", msg->data);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("simple_topic", 10, callback);
    ros::spin(); //ros::spinOnce는 메세지 큐를 한번만 처리함. --> while문과 함께 처리해야한다. 하지만 ros::spin은 무한루프를 형성해서 while이 필요없음

    return 0;
}
