cmake_minimum_required(VERSION 3.5)
project(arm_robots_msgs)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs geometry_msgs trajectory_msgs message_generation)

catkin_python_setup()

add_message_files(
        FILES
        Points.msg
)

add_service_files(
        FILES
        GrippersTrajectory.srv
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        sensor_msgs
        std_msgs
        trajectory_msgs)


catkin_package(CATKIN_DEPENDS geometry_msgs sensor_msgs std_msgs trajectory_msgs message_runtime)
