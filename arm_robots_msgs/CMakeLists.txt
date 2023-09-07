cmake_minimum_required(VERSION 3.5)
project(arm_robots_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs moveit_msgs rospy std_msgs sensor_msgs trajectory_msgs message_generation)

catkin_python_setup()

add_message_files(
        FILES
        GripperConstraint.msg
        Points.msg
        PlanningSceneNoState.msg
)

add_service_files(
        FILES
        SetCDCPDState.srv
        SetGripperConstraints.srv
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        moveit_msgs
        sensor_msgs
        std_msgs
        trajectory_msgs)


catkin_package(CATKIN_DEPENDS geometry_msgs moveit_msgs sensor_msgs std_msgs trajectory_msgs message_runtime)
