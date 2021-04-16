cmake_minimum_required(VERSION 2.8.3)
project(rover_cam)

find_package(catkin REQUIRED
    message_generation
    rospy
    std_msgs
    sensor_msgs
    cv_bridge
    std_srvs
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
    gray_filter.srv
    size_filter.srv
    fps_filter.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
    DEPENDS message_runtime
)
