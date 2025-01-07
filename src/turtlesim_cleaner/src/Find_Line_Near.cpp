#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <turtlesim_cleaner/LineAndNear.h>
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
                   ros::Publisher& line_and_near_pub) {
    std::vector<std::pair<double, double>> points;

    if (msg->data.size() % 2 != 0) {
        ROS_ERROR("Invalid point data: must contain even number of elements (x, y pairs).");
        return;
    }

    for (size_t i = 0; i < msg->data.size(); i += 2) {
        points.emplace_back(msg->data[i], msg->data[i + 1]);
    }

    Line line = calculateLine(points);
    std::pair<double, double> nearest_point = findNearestPoint(line, points);

    turtlesim_cleaner::LineAndNear line_and_near_msg;
    line_and_near_msg.a = line.a;
    line_and_near_msg.b = line.b;
    line_and_near_msg.nearest_point.x = nearest_point.first;
    line_and_near_msg.nearest_point.y = nearest_point.second;
    line_and_near_msg.nearest_point.z = 0.0;

    line_and_near_pub.publish(line_and_near_msg);

    ROS_INFO("Line: y = %.2fx + %.2f", line.a, line.b);
    ROS_INFO("Nearest Point: (%.2f, %.2f)", nearest_point.first, nearest_point.second);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "find_line_near");
    ros::NodeHandle nh;

    ros::Publisher line_and_near_pub = nh.advertise<turtlesim_cleaner::LineAndNear>("line_info_and_near", 10);

    ros::Subscriber point_sub = nh.subscribe<std_msgs::Float32MultiArray>("Points", 10,
        boost::bind(pointCallback, _1, boost::ref(line_and_near_pub)));

    ros::spin();
    return 0;
}
