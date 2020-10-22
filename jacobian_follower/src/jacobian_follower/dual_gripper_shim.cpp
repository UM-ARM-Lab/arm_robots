#include <std_msgs/String.h>

#include <arc_utilities/ros_helpers.hpp>
#include <arc_utilities/eigen_ros_conversions.hpp>

#include <jacobian_follower/dual_gripper_shim.hpp>


std::vector<std::vector<Eigen::Vector3d>> convert(std::vector<arm_robots_msgs::Points> const &grippers)
{
  std::vector<std::vector<Eigen::Vector3d>> stl_grippers;
  for (auto const &gripper : grippers)
  {
    std::vector<Eigen::Vector3d> stl_points;
    for (auto const &point_msg : gripper.points)
    {
      auto const stl_point = ConvertTo<Eigen::Vector3d>(point_msg);
      stl_points.emplace_back(stl_point);
    }
    stl_grippers.emplace_back(stl_points);
  }
  return stl_grippers;
}

DualGripperShim::DualGripperShim(std::string const robot_namespace, ros::NodeHandle const &nh,
                                 ros::NodeHandle const &ph)
    : nh_(nh), ph_(ph),
      execute_traj_srv_(
          nh_.advertiseService("execute_dual_gripper_action", &DualGripperShim::executeDualGripperTrajectory, this)),
      talker_(nh_.advertise<std_msgs::String>("polly", 10, false)),
      traj_goal_time_tolerance_(ROSHelpers::GetParam(ph_, "traj_goal_time_tolerance", 0.05))
{
  auto const default_traj_name = ros::names::append(robot_namespace,
                                                    "both_arms_trajectory_controller/follow_joint_trajectory");
  auto const traj_name = ROSHelpers::GetParam<std::string>(ph,
                                                           "traj_name",
                                                           default_traj_name);
  ROS_INFO_STREAM("waiting for trajectory client " << traj_name);
  trajectory_client_ = std::make_unique<TrajectoryClient>(traj_name, true);
  trajectory_client_->waitForServer();
  ROS_INFO_STREAM("trajectory client connected");
  auto const translation_step_size = ROSHelpers::GetParam<double>(ph_, "translation_step_size", 0.002);
  auto const minimize_rotation = ROSHelpers::GetParam<bool>(ph_, "minimize_rotation", true);
  planner_ = std::make_shared<JacobianFollower>(robot_namespace, translation_step_size, minimize_rotation);
}

bool DualGripperShim::executeDualGripperTrajectory(arm_robots_msgs::GrippersTrajectory::Request &req,
                                                   arm_robots_msgs::GrippersTrajectory::Response &res)
{
  (void) res;
  // Validity checks
  auto const grippers = convert(req.grippers);
  EigenHelpers::VectorQuaterniond preferred_orientations;
  std::transform(req.tool_names.cbegin(), req.tool_names.cend(), std::back_inserter(preferred_orientations), [](auto &&)
  {
    return Eigen::Quaterniond{Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())};
  });
  auto const is_valid = planner_->isRequestValid(req.group_name, req.tool_names, grippers);
  if (not is_valid)
  {
    return true;
  }

  // NOTE: positions are assumed to be in robot_root frame
  auto const n_points = grippers[0].size();
  for (size_t waypoint_idx = 0; waypoint_idx < n_points; ++waypoint_idx)
  {
    PointSequence target_point_sequence;
    for (auto const &gripper : grippers)
    {
      target_point_sequence.emplace_back(gripper[waypoint_idx]);
    }
    auto const traj = planner_->moveInWorldFrame(req.group_name, req.tool_names, preferred_orientations,
                                                 target_point_sequence);
    followJointTrajectory(traj);
  }

  ROS_INFO("Done trajectory");
  return true;
}

void DualGripperShim::followJointTrajectory(robot_trajectory::RobotTrajectory const &traj)
{
  // TODO: make this a param
  if (talk_)
  {
    std_msgs::String executing_action_str;
    executing_action_str.data = "Moving";
    talker_.publish(executing_action_str);
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  moveit_msgs::RobotTrajectory traj_msg;
  traj.getRobotTrajectoryMsg(traj_msg);
  goal.trajectory = traj_msg.joint_trajectory;
  for (const auto &name : goal.trajectory.joint_names)
  {
    control_msgs::JointTolerance tol;
    tol.name = name;
    tol.position = 0.05;
    tol.velocity = 0.5;
    tol.acceleration = 1.0;
    goal.goal_tolerance.push_back(tol);
  }
  goal.goal_time_tolerance = traj_goal_time_tolerance_;

  ROS_INFO("Sending goal ...");
  trajectory_client_->sendGoalAndWait(goal);

  ROS_INFO("Goal finished");
}