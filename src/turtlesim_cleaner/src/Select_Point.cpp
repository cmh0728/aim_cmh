#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h> // 여러 포인트를 다루기 위해 사용
#include <turtlesim_cleaner/LineAndNear.h> // LineAndNear 메시지
#include <cstdlib>
#include <ctime>
#include <vector>

// 글로벌 변수
std::pair<int, int> nearest_point = {0, 0}; // 최초 Nearest Point는 (0, 0)

// 랜덤 포인트 생성 및 퍼블리시 함수
void Random_points_pub(ros::Publisher &publisher, int x, int y, int range, int num_points) {
    // 난수 생성
    std::vector<float> random_points;
    for (int i = 0; i < num_points; ++i) {
        int random_x = x + (std::rand() % (2 * range + 1)) - range;
        int random_y = y + (std::rand() % (2 * range + 1)) - range;
        random_points.push_back(random_x);
        random_points.push_back(random_y);
    }

    // 메시지 생성
    std_msgs::Float32MultiArray points_msg;
    points_msg.data = random_points;

    // 퍼블리시
    publisher.publish(points_msg);

    // 디버깅 출력
    ROS_INFO("Published random points:");
    for (size_t i = 0; i < random_points.size(); i += 2) {
        ROS_INFO("(%.2f, %.2f)", random_points[i], random_points[i + 1]);
    }
}

// line_info_and_near 콜백 함수
void lineAndNearCallback(const turtlesim_cleaner::LineAndNear::ConstPtr& msg) {
    nearest_point.first = static_cast<int>(msg->nearest_point.x);
    nearest_point.second = static_cast<int>(msg->nearest_point.y);
    ROS_INFO("Updated Nearest Point: (%.2f, %.2f)", msg->nearest_point.x, msg->nearest_point.y);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Select_Point");
    ros::NodeHandle nh;

    // 퍼블리셔
    ros::Publisher points_pub = nh.advertise<std_msgs::Float32MultiArray>("Points", 10);

    // 구독자
    ros::Subscriber line_and_near_sub = nh.subscribe("line_info_and_near", 10, lineAndNearCallback);

    // 랜덤 생성 범위 및 포인트 수 설정
    const int range = 10;      // ±10 범위
    const int num_points = 20; // 생성할 랜덤 포인트 개수

    // 랜덤 시드 초기화
    std::srand(std::time(nullptr));

    // 루프 주기 설정 (1/5 Hz = 5초 간격)
    ros::Rate loop_rate(0.2);

    while (ros::ok()) {
        // 랜덤 포인트 생성 및 퍼블리시
        Random_points_pub(points_pub, nearest_point.first, nearest_point.second, range, num_points);

        // ROS 콜백 처리 및 루프 대기
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
