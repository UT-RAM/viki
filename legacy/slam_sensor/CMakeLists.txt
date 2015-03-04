cmake_minimum_required(VERSION 2.8.3)
project(SLAM_sensor)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs
		tf roslib)

include_directories(${catkin_INCLUDE_DIRS} include)

catkin_package(
  DEPENDS
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs
  LIBRARIES
)
add_executable(SLAM_sensor src/main.cpp src/calibrationParser.cpp)
add_dependencies(SLAM_sensor  ${catkin_EXPORTED_TARGETS})
target_link_libraries(SLAM_sensor  ${catkin_LIBRARIES} yaml-cpp)

# Extend with executables for the subtasks, if necessary.