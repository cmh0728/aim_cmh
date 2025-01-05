#include "ros/ros.h"
#include "std_msgs/Int32.h"

void callback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Callback received: %d", msg->data)
}

int main(int argc, char**argv)
{
    ros::init(argc. argv , "sub_with_pub_node")
    ros::NodeHandle nh ;

    ros::Publisher pub = nh.advertise<std_msgs::Int32>("first_pub",10);
    ros::Subscriber sub = nh.subscribe("토픽명",10,callback);

    ros::Rate loop_rate(1);

    while(ros::ok())
    {
        ros::spinonce()
        loop_rate.sleep();
    }
}