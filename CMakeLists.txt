cmake_minimum_required(VERSION 2.8.3)
project(rgbd_camera_calibration)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  std_msgs
  rosbag
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rgbd_camera_calibration
#  CATKIN_DEPENDS rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(filter_bag src/filter_bag.cpp)
target_link_libraries(filter_bag ${catkin_LIBRARIES})
