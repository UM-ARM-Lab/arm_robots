cmake_minimum_required(VERSION 3.10)
project(arm_robots)

find_package(catkin REQUIRED jacobian_follower)

catkin_package(
  CATKIN_DEPENDS jacobian_follower
  )

catkin_python_setup()
