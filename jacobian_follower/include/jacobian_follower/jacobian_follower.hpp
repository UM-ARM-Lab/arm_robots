#pragma once

#include <memory>
#include <string>
#include <vector>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <arc_utilities/eigen_typedefs.hpp>
#include <arc_utilities/moveit_pose_type.hpp>
#include <arm_robots_msgs/Points.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>

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
  moveit_visual_tools::MoveItVisualTools visual_tools_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string const world_frame_;
  std::string const robot_frame_;
  Pose const worldTrobot;
  Pose const robotTworld;

  // For use when moving the EE positions using moveIn[Robot/World]Frame
  double const translation_step_size_;

  trajectory_processing::IterativeParabolicTimeParameterization time_param_;

  bool minimize_rotation_{true};

  explicit JacobianFollower(std::string robot_namespace, double translation_step_size, bool minimize_rotation = true);

  [[nodiscard]] bool isRequestValid(std::string const &group_name,
                                    std::vector<std::string> const &tool_names,
                                    std::vector<std::vector<Eigen::Vector3d>>
                                    const &grippers) const;

  [[nodiscard]] PoseSequence getToolTransforms(std::vector<std::string> const &tool_names,
                                               robot_state::RobotState const &state) const;

  moveit_msgs::RobotTrajectory plan(std::string const &group_name,
                                    std::vector<std::string> const &tool_names,
                                    std::vector<Eigen::Vector4d> const &preferred_tool_orientations,
                                    std::vector<std::vector<Eigen::Vector3d>> const &grippers,
                                    double max_velocity_scaling_factor,
                                    double max_acceleration_scaling_factor);

  robot_trajectory::RobotTrajectory moveInRobotFrame(std::string const &group_name,
                                                     std::vector<std::string> const &tool_names,
                                                     EigenHelpers::VectorQuaterniond const &preferred_tool_orientations,
                                                     PointSequence const &target_tool_positions);

  robot_trajectory::RobotTrajectory moveInWorldFrame(std::string const &group_name,
                                                     std::vector<std::string> const &tool_names,
                                                     EigenHelpers::VectorQuaterniond const &preferred_tool_orientations,
                                                     PointSequence const &target_tool_positions);

  robot_trajectory::RobotTrajectory jacobianPath3d(planning_scene_monitor::LockedPlanningSceneRW &planning_scene,
                                                   moveit::core::JointModelGroup const *jmg,
                                                   std::vector<std::string> const &tool_names,
                                                   EigenHelpers::VectorQuaterniond const &preferred_tool_orientations,
                                                   std::vector<PointSequence> const &tool_paths);

  // Note that robot_goal_points is the target points for the tools, measured in robot frame
  bool jacobianIK(planning_scene_monitor::LockedPlanningSceneRW &planning_scene,
                  moveit::core::JointModelGroup const *jmg,
                  std::vector<std::string> const &tool_names,
                  PoseSequence const &robotTtargets);

  Eigen::VectorXd projectRotationIntoNullspace(Eigen::VectorXd positionCorrectionStep,
                                               Eigen::VectorXd rotationCorrectionStep,
                                               Eigen::MatrixXd const &nullspaceConstraintMatrix,
                                               int const ndof);

  Eigen::MatrixXd getJacobianServoFrame(moveit::core::JointModelGroup const *jmg,
                                        std::vector<std::string> const &tool_names,
                                        robot_state::RobotState const &state, PoseSequence const &robotTservo);

};
