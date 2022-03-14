cmake_minimum_required(VERSION 2.8.3)
project(gimbal_ctrl)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs std_msgs roslaunch)
roslaunch_add_file_check(launch)
include_directories(${catkin_INCLUDE_DIRS} ./include/)
## Declare ROS messages and services

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package

catkin_package()
add_executable(gimbal_ctrl src/gimbal_ctrl.cpp)
add_dependencies(gimbal_ctrl ${${PROJECT_NAME}}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gimbal_ctrl ${catkin_LIBRARIES})
