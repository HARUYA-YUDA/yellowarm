cmake_minimum_required(VERSION 2.8.3)
project(yellowarm)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

add_message_files(
    FILES
    Angles.msg
    Goal_pose.msg
)

generate_messages(
    DEPENDENCIES std_msgs 
)

catkin_package(
    DEPENDS roscpp rospy std_msgs
    CATKIN_DEPENDS message_runtime
)

include_directories(
)

