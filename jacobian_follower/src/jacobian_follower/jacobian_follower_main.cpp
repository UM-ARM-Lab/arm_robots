#include <ros/ros.h>

#include <jacobian_follower/dual_gripper_shim.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "jacobian_follower");

  ros::NodeHandle nh;
  ros::NodeHandle ph("~");

  auto shim = DualGripperShim("hdt_michigan", nh, ph);
  constexpr auto const GREEN{"\033[32m"};
  constexpr auto const RESET{"\033[0m\n"};
  ROS_INFO_STREAM(GREEN << "Ready!" << RESET);

  ros::spin();
  return EXIT_SUCCESS;
}
