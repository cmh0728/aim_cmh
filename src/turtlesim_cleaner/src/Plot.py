# - Plot : Python에서 matplotlib를 이용해서 Point, Line, Nearest Point 실시간 그래프화 

#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# 데이터 저장
points = []
line_info = None
nearest_point = None

# 콜백 함수
def points_callback(msg):
    global points
    # 데이터를 (x, y) 쌍으로 변환
    points = [(msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2)]

def line_callback(msg):
    global line_info
    # 직선 방정식의 기울기(a)와 절편(b) 저장
    line_info = (msg.data[0], msg.data[1])  # a, b

def nearest_callback(msg):
    global nearest_point
    # 가장 가까운 점 저장
    nearest_point = (msg.x, msg.y)

# 플롯 업데이트 함수
def update_plot(frame):
    global points, line_info, nearest_point

    # 플롯 초기화
    plt.cla()

    # 포인트 플롯
    if points:
        x_vals, y_vals = zip(*points)
        plt.scatter(x_vals, y_vals, label="Points", color="blue")

    # 직선 플롯
    if line_info:
        a, b = line_info
        x_line = np.linspace(min(x_vals) - 10, max(x_vals) + 10, 100)
        y_line = a * x_line + b
        plt.plot(x_line, y_line, label=f"Line: y = {a:.2f}x + {b:.2f}", color="red")

    # Nearest Point 플롯
    if nearest_point:
        plt.scatter(nearest_point[0], nearest_point[1], label="Nearest Point", color="green", s=100, marker='*')

    # 플롯 설정
    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Points, Line, and Nearest Point")
    plt.grid(True)

# ROS 노드 초기화 및 메시지 구독
def main():
    rospy.init_node("Plot", anonymous=True)

    # 구독 설정
    rospy.Subscriber("Points", Float32MultiArray, points_callback)
    rospy.Subscriber("line_info", Float32MultiArray, line_callback)
    rospy.Subscriber("near", Point, nearest_callback)

    # 실시간 플롯 설정
    fig = plt.figure()
    ani = FuncAnimation(fig, update_plot, interval=500)  # 500ms마다 업데이트

    plt.show()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
