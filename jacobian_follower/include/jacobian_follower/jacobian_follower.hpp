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
using ConstraintFn =
    std::function<bool(planning_scene::PlanningScenePtr planning_scene, robot_state::RobotState const &state)>;

struct ToolWaypoint {
  Eigen::Vector3d point;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ToolsWaypoint {
  std::vector<ToolWaypoint, Eigen::aligned_allocator<ToolWaypoint>> tools;
};

struct ToolWaypoints {
  std::vector<ToolWaypoint> points;
};

struct ToolsWaypoints {
  std::vector<ToolWaypoints> tools;

  ToolsWaypoint waypoint(std::size_t waypoint_idx) const {
    ToolsWaypoint w;
    std::transform(tools.cbegin(), tools.cend(), std::back_inserter(w.tools),
                   [&](auto const &tool_waypoints) { return tool_waypoints.points[waypoint_idx]; });
    return w;
  }
};

struct JacobianContext {
  planning_scene::PlanningScenePtr planning_scene;
  Pose world_to_robot;
  std::string group_name;
  std::vector<std::string> tool_names;
};

struct JacobianWaypointCommand {
  JacobianContext context;
  ToolsWaypoint tools_waypoint;
  EigenHelpers::VectorQuaterniond preferred_tool_orientations;
  robot_state::RobotState start_state;
};

struct JacobianWaypointsCommand {
  JacobianContext context;
  ToolsWaypoints tools_waypoints;
  EigenHelpers::VectorQuaterniond preferred_tool_orientations;
  robot_state::RobotState start_state;

  JacobianWaypointCommand waypoint(std::size_t waypoint_idx) const {
    return {context, tools_waypoints.waypoint(waypoint_idx), preferred_tool_orientations, start_state};
  }
};

struct JacobianTrajectoryCommand {
  JacobianWaypointsCommand waypoints_command;
  double max_velocity_scaling_factor;
  double max_acceleration_scaling_factor;
};

[[nodiscard]] PoseSequence getToolTransforms(Pose const &world_to_robot, std::vector<std::string> const &tool_names,
                                             robot_state::RobotState const &state);

class JacobianFollower {
 public:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<ros::Publisher> vis_pub_;

  robot_model_loader::RobotModelLoaderPtr model_loader_;
  moveit::core::RobotModelConstPtr const model_;
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

  bool minimize_rotation_;
  ConstraintFn constraint_fun_;

  bool visualize_;
  std::string robot_namespace_;

  explicit JacobianFollower(std::string robot_namespace, double translation_step_size, bool minimize_rotation = true,
                            bool collision_check = true, bool visualize = true);

  bool connect_to_psm();

  [[nodiscard]] bool isRequestValid(JacobianWaypointsCommand waypoints_command) const;

  PlanResultMsg plan(std::string const &group_name, std::vector<std::string> const &tool_names,
                     std::vector<Eigen::Vector4d> const &preferred_tool_orientations,
                     std::vector<PointSequence> const &grippers, double max_velocity_scaling_factor,
                     double max_acceleration_scaling_factor);

  PlanResultMsg plan(std::string const &group_name, std::vector<std::string> const &tool_names,
                     std::vector<Eigen::Vector4d> const &preferred_tool_orientations,
                     moveit_msgs::RobotState const &start_state_msg, std::vector<PointSequence> const &grippers,
                     double max_velocity_scaling_factor, double max_acceleration_scaling_factor);

  PlanResultMsg plan(std::string const &group_name, std::vector<std::string> const &tool_names,
                     std::vector<Eigen::Vector4d> const &preferred_tool_orientations,
                     moveit_msgs::RobotState const &start_state_msg, moveit_msgs::PlanningScene const &scene_msg,
                     std::vector<PointSequence> const &grippers, double max_velocity_scaling_factor,
                     double max_acceleration_scaling_factor);

  // TODO: Return std::vector<std_msgs/JointState.msg> to avoid ambiguity in the joint ordering?
  std::vector<std::vector<double>> compute_IK_solutions(geometry_msgs::Pose target_pose,
                                                        const std::string &group_name) const;

  // TODO: Accept std_msgs/JointState.msg to avoid ambiguity in the joint ordering?
  geometry_msgs::Pose computeFK(const std::vector<double> &joint_angles, const std::string &group_name) const;

  PlanResult plan(JacobianTrajectoryCommand traj_command);

  PlanResult moveInWorldFrame(JacobianWaypointCommand waypoint_command);

  robot_trajectory::RobotTrajectory jacobianPath3d(planning_scene::PlanningScenePtr planning_scene,
                                                   Pose const &world_to_robot, moveit::core::JointModelGroup const *jmg,
                                                   std::vector<std::string> const &tool_names,
                                                   EigenHelpers::VectorQuaterniond const &preferred_tool_orientations,
                                                   std::vector<PointSequence> const &tools_waypoint_interpolated);

  // Note that robot_goal_points is the target points for the tools, measured in robot frame
  bool jacobianIK(planning_scene::PlanningScenePtr planning_scene, Pose const &world_to_robot,
                  moveit::core::JointModelGroup const *jmg, std::vector<std::string> const &tool_names,
                  PoseSequence const &robotTtargets, const ConstraintFn &constraint_fn);

  Eigen::VectorXd projectRotationIntoNullspace(Eigen::VectorXd positionCorrectionStep,
                                               Eigen::VectorXd rotationCorrectionStep,
                                               Eigen::MatrixXd const &nullspaceConstraintMatrix, int ndof);

  Eigen::MatrixXd getJacobianServoFrame(moveit::core::JointModelGroup const *jmg,
                                        std::vector<std::string> const &tool_names,
                                        robot_state::RobotState const &state, PoseSequence const &robotTservo);

  collision_detection::CollisionResult checkCollision(planning_scene::PlanningScenePtr planning_scene,
                                                      robot_state::RobotState const &state);

  bool check_collision(moveit_msgs::PlanningScene const &scene_msg, moveit_msgs::RobotState const &start_state);

  PointSequence get_tool_positions(std::vector<std::string> tool_names, moveit_msgs::RobotState const &state_msg);

  void debugLogState(std::string prefix, robot_state::RobotState const &state);

  Eigen::Matrix4Xd getLinkToRobotTransform(std::vector<std::string> joint_names, std::vector<double> joint_positions,
                                           std::string link_name);

  std::vector<Eigen::Matrix4Xd> getLinkToRobotTransforms(std::vector<std::string> const &joint_names,
                                                         std::vector<double> const &joint_positions,
                                                         robot_state::RobotStatePtr robot_state) const;

  std::vector<Eigen::Matrix4Xd> getLinkToRobotTransforms(std::vector<std::string> const &joint_names,
                                                         std::vector<double> const &joint_positions) const;

  std::vector<std::vector<Eigen::Matrix4Xd>> batchGetLinkToRobotTransforms(
      std::vector<std::vector<std::string>> const &joint_names,
      std::vector<std::vector<double>> const &joint_positions) const;

  std::vector<std::string> getLinkNames()const ;
};
