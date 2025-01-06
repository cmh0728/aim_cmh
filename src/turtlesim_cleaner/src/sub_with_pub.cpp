#include "ros/ros.h"
#include "turtlesim_cleaner/MyCustom.h"
#include "std_msgs/Bool.h"

bool find_obj_status = false; // 글로벌 변수로 상태 유지

void callback(const turtlesim_cleaner::MyCustom::ConstPtr& msg) {
    ROS_INFO("Received: find_msg=%s",  msg->find_msg ? "true" : "false");
    find_obj_status = msg->find_msg; // 상태 업데이트
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sub_with_pub_node");
    ros::NodeHandle nh;

    // Publisher 설정 조건에 맞게 되면 control msg 토픽 쏴주기
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("control_msg", 10);
    std_msgs::Bool control_msg; //control_msg변수선언

    // Subscriber 설정 내가 만든 custom topic 받아옴
    ros::Subscriber sub = nh.subscribe("custom_topic", 10, callback);

    ros::Rate loop_rate(5);

    while (ros::ok()) {

        // find_msg 상태에 따라 control_msg 값 변경
        if (find_obj_status) 
        {
            control_msg.data = true;  // 예: 장애물 발견 시 true
            ROS_INFO("object detect!!!! stop!!!!!!!!1");
            pub.publish(control_msg); //마지막으로 쏴주고 종료되게. 이게 없어서 업데이트가 안되고 종료됨
        }
        else 
        {
            control_msg.data = false; // 예: 정상 진행 시 false
        }

        pub.publish(control_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
