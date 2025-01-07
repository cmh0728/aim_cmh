// Select Point : line info & near의 Nearest Point에 대해서 +- 10 범위의 랜덤한 정수를 20개 생성한 후 points 토픽으로 1/5 Hz pub

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h> //여러개의 포인트를 다루기 위해서 
#include <cstdlib>
#include <ctime>
#include <vector>

// 랜덤 포인트 생성 , pub
void Random_points_pub(ros::Publisher &publisher, int nearest_point, int range, int num_points)
{
    // 난수생성
    std::vector<float> random_points;
    for (int i = 0; i < num_points; ++i)
    {
        int random_value = nearest_point + (std::rand() % (2 * range + 1)) - range;
        random_points.push_back(random_value);
    }

    // 메시지 생성
    std_msgs::Float32MultiArray points_msg;
    points_msg.data = random_points;

    // 퍼블리시
    publisher.publish(points_msg);

    // 디버깅 출력
    ROS_INFO("Published random points:");
    for (float point : random_points)
    {
        ROS_INFO("%.2f", point);
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Select_Point");
    ros::NodeHandle nh;

    ros::Publisher points_pub = nh.advertise<std_msgs::Float32MultiArray>("Points", 10);

    // Nearest Point와 랜덤 생성 범위 설정
    const int nearest_point = 30; // 예제: Nearest Point 값
    const int range = 10;         // ±10 범위
    const int num_points = 20;    // 생성할 랜덤 포인트 개수

    // 랜덤 시드 초기화
    std::srand(std::time(nullptr));

    // 루프 주기 설정 (1/5 Hz = 5초 간격)
    ros::Rate loop_rate(0.2);

    while (ros::ok())
    {
        // 랜덤 포인트 생성 및 퍼블리시
        Random_points_pub(points_pub, nearest_point, range, num_points);

        // 루프 대기
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
