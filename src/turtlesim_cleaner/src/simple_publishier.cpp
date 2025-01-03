#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
// 헤더파일 포함 다양한 라이브러리를 가져와서 함수를 사용하기 위함 


//int main 은 main함수로, 코드에서 프로그램의 진입점. 실행되는 부분이다. argc는 명령줄의 개수, argv는 명령줄 인자들의 문자열 배열 형태 저장, 각 원소는 포인터 
int main(int argc, char **argv) {

    // std::cout << "argc : " <<argc << argv[0];

    ros::init(argc, argv, "simple_publisher"); //노드 초기화, argc와 argv는 ros master와 통신을 위함  
    ros::NodeHandle nh; // nh라는 노드 핸들 생성 using namespace 와 비슷하게, nh라는 거로 정의해놓고 사용.

    ros::Publisher pub = nh.advertise<std_msgs::Int32>("simple_topic", 10); //nh.adevertise<T> 에서 T는 메세지 타입. 퍼블리셔 객체를 생성, 10은 큐 사이즈 

    ros::Rate loop_rate(1); //루프 레이트 설정. 10은 초당 10Hz
    int count = 0;
    while (ros::ok())  // ros::ok는 ros 노드가 활성화상태인지 확인. 이게 없으면 ctrl+c로 종료가 안됨.
    { 
        std_msgs::Int32 msg;
        msg.data = count;
        pub.publish(msg);

        ROS_INFO("Published: %d", msg.data); //ros의 log 함수 정보를 출력하는 함수 
        count++;
        loop_rate.sleep(); // 위에서 설정한 rate에 맞게 대기 
    }

    return 0; // 위의 while루프가 종료되면 종료 
}
