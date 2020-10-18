#pragma once

#include <memory>

#include <actionlib/client/simple_action_client.h>
#include <arc_utilities/listener.hpp>
#include <arm_robots_msgs/GrippersTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <jacobian_follower/jacobian_follower.hpp>

class DualGripperShim
{
 public:
  bool talk_{false};
  ros::NodeHandle nh_;
  ros::NodeHandle ph_;
  std::shared_ptr<JacobianFollower> planner_;
  ros::ServiceServer execute_traj_srv_;
  ros::Publisher talker_;
  using TrajectoryClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;
  std::unique_ptr<TrajectoryClient> trajectory_client_;
  ros::Duration const traj_goal_time_tolerance_;

  DualGripperShim(std::string robot_namespace, const ros::NodeHandle &nh, const ros::NodeHandle &ph);


  bool executeDualGripperTrajectory(arm_robots_msgs::GrippersTrajectory::Request &req,
                                    arm_robots_msgs::GrippersTrajectory::Response &res);

  void followJointTrajectory(robot_trajectory::RobotTrajectory const &traj);
};