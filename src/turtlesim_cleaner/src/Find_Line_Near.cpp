// - Find Line & Near : 받은 point 토픽에 대해 Line 생성(최소자승법) 후 Line에 대해 Point 중 Nearest Point 찾고 line info & near 토픽으로 pub 

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <cmath>
#include <limits>

// 직선 방정식을 나타내는 구조체
struct Line {
    double a; // 기울기
    double b; // y절편
};

// 최소자승법으로 직선 생성
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
        ROS_ERROR("Cannot calculate line: points are collinear or insufficient.");
        return {0.0, 0.0}; // Default values
    }

    double a = (n * sum_xy - sum_x * sum_y) / denominator;
    double b = (sum_y * sum_xx - sum_x * sum_xy) / denominator;

    return {a, b};
}

// 직선과 점 간의 수직 거리 계산
double calculateDistanceToLine(const Line& line, double x, double y) {
    double numerator = std::abs(line.a * x - y + line.b);
    double denominator = std::sqrt(line.a * line.a + 1.0);
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
                   ros::Publisher& line_pub,
                   ros::Publisher& near_pub) {
    std::vector<std::pair<double, double>> points;

    // 입력 데이터를 (x, y) 쌍으로 변환
    if (msg->data.size() % 2 != 0) {
        ROS_ERROR("Invalid point data: must contain even number of elements (x, y pairs).");
        return;
    }
    for (size_t i = 0; i < msg->data.size(); i += 2) {
        points.emplace_back(msg->data[i], msg->data[i + 1]);
    }

    // 직선 계산
    Line line = calculateLine(points);

    // 가장 가까운 점 계산
    std::pair<double, double> nearest_point = findNearestPoint(line, points);

    // Line 정보 퍼블리시 (기울기 a, 절편 b)
    std_msgs::Float32MultiArray line_msg;
    line_msg.data.push_back(line.a);
    line_msg.data.push_back(line.b);
    line_pub.publish(line_msg);

    // 가장 가까운 점 퍼블리시 (x, y)
    geometry_msgs::Point near_msg;
    near_msg.x = nearest_point.first;
    near_msg.y = nearest_point.second;
    near_msg.z = 0.0; // z는 사용하지 않음
    near_pub.publish(near_msg);

    // 디버깅 출력
    ROS_INFO("Line: y = %.2fx + %.2f", line.a, line.b);
    ROS_INFO("Nearest Point: (%.2f, %.2f)", nearest_point.first, nearest_point.second);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Find_Line_Near");
    ros::NodeHandle nh;

    // 퍼블리셔 생성
    ros::Publisher line_pub = nh.advertise<std_msgs::Float32MultiArray>("line_info", 10);
    ros::Publisher near_pub = nh.advertise<geometry_msgs::Point>("near", 10);

    // 구독자 생성
    ros::Subscriber point_sub = nh.subscribe<std_msgs::Float32MultiArray>("Points", 10,
        boost::bind(pointCallback, _1, boost::ref(line_pub), boost::ref(near_pub)));

    ros::spin();
    return 0;
}
