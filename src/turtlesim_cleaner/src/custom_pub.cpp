#include "ros/ros.h"
#include "turtlesim_cleaner/MyCustom.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "custom_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<turtlesim_cleaner::MyCustom>("custom_topic", 5);
    ros::Rate loop_rate(5);

    float count = 0.0 ; 
    turtlesim_cleaner::MyCustom msg; // 메모리 문제떄문에 while 루프 밖에서 선언해주는것이 좋음+ 쓰레기값 ( 초기화가 안되서)
    msg.y = 1.0;
    msg.arr = {2.0, 3.0, 4.0};

    while (ros::ok())
    {

        msg.x = count ;

        ROS_INFO("Publishing: x=%f, y=%f, arr[0]=%f", msg.x, msg.y, msg.arr[0]);
        pub.publish(msg);

        count ++ ;
        loop_rate.sleep();
    }
    return 0;
 
}
