#pragma once

#include <memory>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <arm_robots_msgs/GrippersTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <jacobian_follower/listener.hpp>
#include <jacobian_follower/jacobian_follower.hpp>

class DualGripperShim
{
 public:
  ros::NodeHandle nh_;
  ros::NodeHandle ph_;
  std::shared_ptr<JacobianFollower> planner_;
  ros::ServiceServer execute_traj_srv_;
  ros::Publisher talker_;
  using TrajectoryClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;
  std::unique_ptr<TrajectoryClient> trajectory_client_;
  ros::Duration const traj_goal_time_tolerance_;

  DualGripperShim(const ros::NodeHandle &nh, const ros::NodeHandle &ph);


  bool executeDualGripperTrajectory(arm_robots_msgs::GrippersTrajectory::Request &req,
                                    arm_robots_msgs::GrippersTrajectory::Response &res);

  void followJointTrajectory(trajectory_msgs::JointTrajectory const &traj);
};