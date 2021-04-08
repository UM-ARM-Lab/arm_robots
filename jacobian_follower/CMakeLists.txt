cmake_minimum_required(VERSION 3.10)
project(jacobian_follower)

set(CATKIN_PACKAGES
        arc_utilities
        moveit_ros_planning
        arm_robots_msgs
        roscpp
        tf2_ros
        tf2_msgs
        pyrosmsg
        moveit_visual_tools
        pybind11_catkin
        eigen_conversions
        )

find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGES}
        )

catkin_python_setup()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES jacobian_follower
        CATKIN_DEPENDS ${CATKIN_PACKAGES}
)

set(FLAGS -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror=return-type -Werror=missing-field-initializers)

add_library(jacobian_follower
        src/jacobian_follower/jacobian_utils.cpp
        src/jacobian_follower/jacobian_follower.cpp)
target_include_directories(jacobian_follower PUBLIC include)
target_include_directories(jacobian_follower SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(jacobian_follower PUBLIC ${catkin_LIBRARIES})
target_compile_options(jacobian_follower PUBLIC ${FLAGS})

add_executable(jacobian_follower_main
        src/jacobian_follower/jacobian_follower_main.cpp
        src/jacobian_follower/dual_gripper_shim.cpp)
target_link_libraries(jacobian_follower_main PUBLIC jacobian_follower)
set_target_properties(jacobian_follower_main PROPERTIES OUTPUT_NAME "jacobian_follower")

pybind_add_module(pyjacobian_follower src/jacobian_follower/bindings.cpp)
target_link_libraries(pyjacobian_follower PUBLIC jacobian_follower)