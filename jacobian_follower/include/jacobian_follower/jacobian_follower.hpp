#pragma once

#include <arm_robots_msgs/Points.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <arc_utilities/eigen_typedefs.hpp>
#include <arc_utilities/moveit_pose_type.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using PlanResult = std::pair<robot_trajectory::RobotTrajectory, bool>;
using PlanResultMsg = std::pair<moveit_msgs::RobotTrajectory, bool>;

[[nodiscard]] PoseSequence getToolTransforms(Pose const &world_to_robot, std::vector<std::string> const &tool_names,
                                             robot_state::RobotState const &state);

class JacobianFollower {
 public:
  enum { NeedsToAlign = ((sizeof(Pose) % 16) == 0) };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<ros::Publisher> vis_pub_;

  robot_model_loader::RobotModelLoaderPtr model_loader_;
  robot_model::RobotModelPtr model_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;

  // Debugging
  moveit_visual_tools::MoveItVisualTools visual_tools_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string const world_frame_;
  std::string const robot_frame_;

  // For use when moving the EE positions using moveIn[Robot/World]Frame
  double const translation_step_size_;

  trajectory_processing::IterativeParabolicTimeParameterization time_param_;

  bool minimize_rotation_{true};

  explicit JacobianFollower(std::string robot_namespace, double translation_step_size, bool minimize_rotation = true);

  [[nodiscard]] bool isRequestValid(std::string const &group_name, std::vector<std::string> const &tool_names,
                                    std::vector<std::vector<Eigen::Vector3d>> const &grippers) const;

  PlanResultMsg plan_return_msg(std::string const &group_name, std::vector<std::string> const &tool_names,
                                std::vector<Eigen::Vector4d> const &preferred_tool_orientations,
                                moveit_msgs::RobotState const &start_state,
                                std::vector<std::vector<Eigen::Vector3d>> const &grippers,
                                double max_velocity_scaling_factor, double max_acceleration_scaling_factor);

  PlanResultMsg plan_return_msg(std::string const &group_name, std::vector<std::string> const &tool_names,
                                std::vector<Eigen::Vector4d> const &preferred_tool_orientations,
                                std::vector<std::vector<Eigen::Vector3d>> const &grippers,
                                double max_velocity_scaling_factor, double max_acceleration_scaling_factor);

  PlanResult plan(std::string const &group_name, std::vector<std::string> const &tool_names,
                  std::vector<Eigen::Vector4d> const &preferred_tool_orientations,
                  std::vector<std::vector<Eigen::Vector3d>> const &grippers, double max_velocity_scaling_factor,
                  double max_acceleration_scaling_factor);

  PlanResult plan(planning_scene_monitor::LockedPlanningSceneRW &planning_scene, std::string const &group_name,
                  std::vector<std::string> const &tool_names,
                  std::vector<Eigen::Vector4d> const &preferred_tool_orientations,
                  robot_state::RobotState const &start_state, std::vector<std::vector<Eigen::Vector3d>> const &grippers,
                  double max_velocity_scaling_factor, double max_acceleration_scaling_factor);

  // TODO: Return std::vector<std_msgs/JointState.msg> to avoid ambiguity in the joint ordering?
  std::vector<std::vector<double>> compute_IK_solutions(geometry_msgs::Pose target_pose, const std::string &group_name);

  // TODO: Accept std_msgs/JointState.msg to avoid ambiguity in the joint ordering?
  geometry_msgs::Pose computeFK(const std::vector<double> &joint_angles, const std::string &group_name);

  PlanResult moveInRobotFrame(planning_scene_monitor::LockedPlanningSceneRW &planning_scene,
                              std::string const &group_name, std::vector<std::string> const &tool_names,
                              EigenHelpers::VectorQuaterniond const &preferred_tool_orientations,
                              robot_state::RobotState const &start_state, PointSequence const &target_tool_positions);

  PlanResult moveInWorldFrame(planning_scene_monitor::LockedPlanningSceneRW &planning_scene,
                              std::string const &group_name, std::vector<std::string> const &tool_names,
                              EigenHelpers::VectorQuaterniond const &preferred_tool_orientations,
                              robot_state::RobotState const &start_state, PointSequence const &target_tool_positions);

  robot_trajectory::RobotTrajectory jacobianPath3d(planning_scene_monitor::LockedPlanningSceneRW &planning_scene,
                                                   Pose const &world_to_robot, moveit::core::JointModelGroup const *jmg,
                                                   std::vector<std::string> const &tool_names,
                                                   EigenHelpers::VectorQuaterniond const &preferred_tool_orientations,
                                                   std::vector<PointSequence> const &tool_paths);

  // Note that robot_goal_points is the target points for the tools, measured in robot frame
  bool jacobianIK(planning_scene_monitor::LockedPlanningSceneRW &planning_scene, Pose const &world_to_robot,
                  moveit::core::JointModelGroup const *jmg, std::vector<std::string> const &tool_names,
                  PoseSequence const &robotTtargets);

  Eigen::VectorXd projectRotationIntoNullspace(Eigen::VectorXd positionCorrectionStep,
                                               Eigen::VectorXd rotationCorrectionStep,
                                               Eigen::MatrixXd const &nullspaceConstraintMatrix, int const ndof);

  Eigen::MatrixXd getJacobianServoFrame(moveit::core::JointModelGroup const *jmg,
                                        std::vector<std::string> const &tool_names,
                                        robot_state::RobotState const &state, PoseSequence const &robotTservo);

  void debugLogState(std::string prefix, robot_state::RobotState const &state);
};
