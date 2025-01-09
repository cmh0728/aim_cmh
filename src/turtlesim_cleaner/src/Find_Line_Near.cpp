#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <turtlesim_cleaner/LineAndNear.h>

//최소자승법을 위한 헤더파일 cpp표준 
#include <vector> // 동적배열
#include <cmath> //수학함수
#include <limits> //최대.최솟값 사용 


//최소자승법이 맞는지 확인하기 . 

// 직선 방정식을 나타내는 구조체--> 밑의 함수 정의를 위해서 (커스텀 자료형)
struct Line {
    double a1; // 기울기
    double a0; // y절편
};

// 선형 (1차 )최소자승법으로 직선 생성 --> Line은 사용자 정의 데이터 타입 , 
Line calculateLine(const std::vector<std::pair<double, double>>& points) {
    double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
    size_t n = points.size();

    for (const auto& point : points) {
        sum_x += point.first;
        sum_y += point.second;
        sum_xx += point.first * point.first;
        sum_xy += point.first * point.second;
    }

    double denominator = n * sum_xx - sum_x * sum_x;
    if (denominator == 0.0) {
        return {0.0, 0.0}; // Default values
    }

    double a1 = (n * sum_xy - sum_x * sum_y) / denominator;
    double a0 = (sum_y * sum_xx - sum_x * sum_xy) / denominator; // least-squares 수치해석 curve fitting 확인 

    return {a1, a0};
}

// 직선과 점 간의 수직 거리 계산
double calculateDistanceToLine(const Line& line, double x, double y) 
{
    double numerator = std::abs(line.a1 * x - y + line.a0);
    double denominator = std::sqrt(line.a1 * line.a1 + 1.0);
    return numerator / denominator;
}

// 가장 가까운 점 찾기
std::pair<double, double> findNearestPoint(const Line& line, const std::vector<std::pair<double, double>>& points) {
    double min_distance = std::numeric_limits<double>::max();
    std::pair<double, double> nearest_point = {0.0, 0.0};

    for (const auto& point : points) {
        double distance = calculateDistanceToLine(line, point.first, point.second);
        if (distance < min_distance) {
            min_distance = distance;
            nearest_point = point;
        }
    }

    return nearest_point;
}

// Point 데이터 수신 콜백 함수
void pointCallback(const std_msgs::Float32MultiArray::ConstPtr& msg,
                   ros::Publisher& line_and_near_pub) {
    std::vector<std::pair<double, double>> points;

    if (msg->data.size() % 2 != 0) {
        return;
    }

    for (size_t i = 0; i < msg->data.size(); i += 2) {
        points.emplace_back(msg->data[i], msg->data[i + 1]);
    }

    Line line = calculateLine(points);
    std::pair<double, double> nearest_point = findNearestPoint(line, points);

    turtlesim_cleaner::LineAndNear line_and_near_msg;
    line_and_near_msg.a = line.a1;
    line_and_near_msg.b = line.a0;
    line_and_near_msg.nearest_point.x = nearest_point.first;
    line_and_near_msg.nearest_point.y = nearest_point.second;
    line_and_near_msg.nearest_point.z = 0.0;

    line_and_near_pub.publish(line_and_near_msg); // 계산한 결과를 바로 pub. 이벤트 기반에 좋음(실시간성)

    ROS_INFO("Line: y = %.2fx + %.2f", line.a1, line.a0); // a가 a1, b가 a0
    ROS_INFO("Nearest Point: (%.2f, %.2f)", nearest_point.first, nearest_point.second);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "Find_Line_Near");
    ros::NodeHandle nh;

    ros::Publisher line_and_near_pub = nh.advertise<turtlesim_cleaner::LineAndNear>("LineInfo_And_Near", 10);

    ros::Subscriber point_sub = nh.subscribe<std_msgs::Float32MultiArray>("Points", 10,
        boost::bind(pointCallback, _1, boost::ref(line_and_near_pub)));

    ros::spin();
    return 0;
}
