cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_cleaner)

#패키지 의존성 
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  turtlesim
  message_generation
)

## Python 헤더 및 라이브러리 경로
find_package(Python3 REQUIRED COMPONENTS Development)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MyCustom.msg
 )

## C++11 지원 설정
add_compile_options(-std=c++11)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs 
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlesim_cleaner
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs turtlesim
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime

)

## Include 디렉토리 설정
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /home/minhyeok/aim_ws/src/matplotlib-cpp  # matplotlibcpp.h 위치
  ${Python3_INCLUDE_DIRS}                   # Python 헤더 파일 경로
)


## 실행 파일 생성
add_executable(robot_cleaner_node src/robot_cleaner.cpp)

## 링킹
target_link_libraries(robot_cleaner_node
  ${catkin_LIBRARIES}
)

## 실행 파일 생성
add_executable(simple_publisher src/simple_publishier.cpp)

## 링킹
target_link_libraries(simple_publisher
  ${catkin_LIBRARIES}
)

## 실행 파일 생성
add_executable(simple_subscriber src/simple_subscriber.cpp)

## 링킹
target_link_libraries(simple_subscriber
  ${catkin_LIBRARIES}
)


#커스텀 메세지 관련

add_executable(custom_publisher src/custom_pub.cpp)
add_dependencies(custom_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(custom_publisher ${catkin_LIBRARIES})

add_executable(custom_subscriber src/custom_sub.cpp)
add_dependencies(custom_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(custom_subscriber ${catkin_LIBRARIES})


#pub+sub관련
add_executable(sub_with_pub_node src/sub_with_pub.cpp)
add_dependencies(sub_with_pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sub_with_pub_node ${catkin_LIBRARIES})

## matplotlib_example
add_executable(matplot_example src/matplot_example.cpp)
target_link_libraries(matplot_example
  ${catkin_LIBRARIES}
  ${Python3_LIBRARIES}  # Python 라이브러리 경로
)