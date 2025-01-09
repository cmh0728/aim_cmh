#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h> // 여러 포인트를 다루기 위해 배열 사용 
#include <turtlesim_cleaner/LineAndNear.h> // custom msg
#include <cstdlib>
#include <ctime>
#include <vector>

// 전역변수 설정 nearest_point : 가장 가까운 포인트를 저장 --> 콜백함수에서 받아온 뒤 줘야하기 때문에 
std::pair<int, int> nearest_point = {0, 0}; // 최초 Nearest Point를 (0, 0)으로 초기화


// 퍼블리셔 함수 
void Random_points_pub(ros::Publisher &publisher, int x, int y, int range, int num_points) 
{
    // 난수로 생성된 포인트를 저장할 벡터
    std::vector<float> random_points;

    // num_points개의 포인트를 생성
    for (int i = 0; i < num_points; ++i) {

        // 난수 생성 검토 필요 
        // x와 y 좌표에 대해 범위 내에서 난수를 생성
        int random_x = x + (std::rand() % (2 * range + 1)) - range; //std네임스페이스 안에 random함수 사용 
        int random_y = y + (std::rand() % (2 * range + 1)) - range;

        // 생성된 좌표를 벡터에 추가
        random_points.push_back(random_x); //push back함수는 벡터의 끝에 새로운 요소를 추가 (append와 비슷한 기능 ) --> cpp에서 vector는 list와 비슷 
        random_points.push_back(random_y);
    }

    // points_msg객체 선언 및 pub
    std_msgs::Float32MultiArray points_msg;
    points_msg.data = random_points; //난수를 메세지에 넣기 
    publisher.publish(points_msg);

    // 정상작동 출력확인
    ROS_INFO("Published random points");
}


// 콜백함수 설정 
void lineAndNearCallback(const turtlesim_cleaner::LineAndNear::ConstPtr& msg) {
    // 수신된 메시지의 nearest_point 좌표를 업데이트
    nearest_point.first = static_cast<int>(msg->nearest_point.x);
    nearest_point.second = static_cast<int>(msg->nearest_point.y);

    // 업데이트된 좌표를 출력 
    ROS_INFO("Updated Nearest Point: (%.2f, %.2f)", msg->nearest_point.x, msg->nearest_point.y);
}

int main(int argc, char **argv) {
    // 노드 선언 및 초기화 
    ros::init(argc, argv, "Select_Point"); //노드명 띄어쓰기 넣으려면 리매핑 필요 --> 일단 패스 
    ros::NodeHandle nh;

    ros::Publisher points_pub = nh.advertise<std_msgs::Float32MultiArray>("Points", 10); //Points 토픽으로 pub
    ros::Subscriber line_and_near_sub = nh.subscribe("LineInfo_And_Near", 10, lineAndNearCallback); //Line_info_and_Near 토픽 sub


    const int range = 10;      // 포인트 생성 범위 (상수--> 변경 불가)
    const int num_points = 20; // 포인트 개수 

    // 난수 생성기를 위한 시드 초기화
    std::srand(std::time(nullptr)); // --> 현재 시간을 초 단위로 반환하는것을 시드로, 항상 다른 난수 생성을 보장 

    // 루프 주기 설정 (1/5 Hz = 5초 간격)
    ros::Rate loop_rate(0.2);

    while (ros::ok()) {
        // pub 함수 실행 
        Random_points_pub(points_pub, nearest_point.first, nearest_point.second, range, num_points);

        // ROS 콜백 함수 호출
        ros::spinOnce();

        // 주기적으로 대기
        loop_rate.sleep();
    }

    return 0;
}
