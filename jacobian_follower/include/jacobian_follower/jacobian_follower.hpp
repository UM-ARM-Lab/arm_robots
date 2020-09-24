#pragma once

#include <memory>
#include <string>
#include <vector>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <arm_robots_msgs/Points.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <arc_utilities/eigen_typedefs.hpp>

#include <jacobian_follower//moveit_pose_type.hpp>

using Matrix6Xd = Eigen::Matrix<double, 6, Eigen::Dynamic>;

class JacobianFollower
{
 public:
  enum
  {
    NeedsToAlign = ((sizeof(Pose) % 16) == 0)
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<ros::Publisher> vis_pub_;

  robot_model_loader::RobotModelLoaderPtr model_loader_;
  robot_model::RobotModelPtr model_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;

  // Debugging
  std::function<void(visualization_msgs::MarkerArray)> waypoints_vis_callback_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string const world_frame_;
  std::string const robot_frame_;
  Pose const worldTrobot;
  Pose const robotTworld;

  // Keep a "desired" orientation so we can try to preserve this orientation while servoing (optionally)
  EigenHelpers::VectorQuaterniond nominal_tool_orientations_;

  // For use when moving the EE positions using moveIn[Robot/World]Frame
  double const translation_step_size_;

  bool minimize_rotation_{true};

  JacobianFollower(double translation_step_size, bool minimize_rotation = true);

  void setWaypointsVisCallback(std::function<void(visualization_msgs::MarkerArray)> waypoints_vis_callback);

  bool isRequestValid(std::string const &group_name,
                      std::vector<std::string> const &tool_names,
                      std::vector<std::vector<Eigen::Vector3d>>
                      const &grippers,
                      double speed) const;


  void setDesiredToolOrientations(std::vector<std::string> const &tool_names,
                                  EigenHelpers::VectorQuaterniond const &nominal_tool_orientations);

  PoseSequence getToolTransforms(std::vector<std::string> const &tool_names,
                                 robot_state::RobotState const &state) const;

  moveit_msgs::RobotTrajectory plan(std::string const &group_name,
                                    std::vector<std::string> const &tool_names,
                                    std::vector<std::vector<Eigen::Vector3d>> const &grippers,
                                    double speed);

  trajectory_msgs::JointTrajectory moveInRobotFrame(std::string const &group_name,
                                                    std::vector<std::string> const &tool_names,
                                                    PointSequence const &target_tool_positions,
                                                    double speed);

  trajectory_msgs::JointTrajectory moveInWorldFrame(std::string const &group_name,
                                                    std::vector<std::string> const &tool_names,
                                                    PointSequence const &target_tool_positions,
                                                    double speed);

  trajectory_msgs::JointTrajectory jacobianPath3d(planning_scene_monitor::LockedPlanningSceneRW &planning_scene,
                                                  moveit::core::JointModelGroup const *jmg,
                                                  std::vector<std::string> const &tool_names,
                                                  std::vector<PointSequence> const &tool_paths,
                                                  double seconds_per_step);

  // Note that robot_goal_points is the target points for the tools, measured in robot frame
  bool jacobianIK(planning_scene_monitor::LockedPlanningSceneRW &planning_scene,
                  moveit::core::JointModelGroup const *jmg,
                  std::vector<std::string> const &tool_names,
                  PoseSequence const &robotTtargets);

  Eigen::MatrixXd getJacobianServoFrame(moveit::core::JointModelGroup const *jmg,
                                        std::vector<std::string> const &tool_names,
                                        robot_state::RobotState const &state, PoseSequence const &robotTservo);

  Matrix6Xd getJacobianServoFrame(moveit::core::JointModelGroup const *jmg,
                                  robot_state::RobotState const &state,
                                  robot_model::LinkModel const *link,
                                  Pose const &robotTservo) const;

};
