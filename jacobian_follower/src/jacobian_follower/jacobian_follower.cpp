#include <arc_utilities/enumerate.h>
#include <bio_ik/bio_ik.h>
#include <moveit/dynamics_solver/dynamics_solver.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/String.h>

#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/eigen_ros_conversions.hpp>
#include <arc_utilities/eigen_transforms.hpp>
#include <arc_utilities/moveit_ostream_operators.hpp>
#include <arc_utilities/moveit_pose_type.hpp>
#include <arc_utilities/ostream_operators.hpp>
#include <arc_utilities/ros_helpers.hpp>
#include <boost/range/combine.hpp>
#include <exception>
#include <jacobian_follower/jacobian_follower.hpp>
#include <jacobian_follower/jacobian_utils.hpp>
#include <sstream>

namespace gm = geometry_msgs;
using ColorBuilder = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>;
using ArrayXb = Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic>;
using VecArrayXb = Eigen::Array<bool, Eigen::Dynamic, 1>;

constexpr auto const LOGGER_NAME{"JacobianFollower"};

PoseSequence getToolTransforms(Pose const &world_to_robot, std::vector<std::string> const &tool_names,
                               robot_state::RobotState const &state) {
  auto get_tool_pose = [&](std::string const &tool_name) {
    // the results of getGlobalLinkTransform is in the same moveit "model frame", as given by
    // RobotModel::getModelFrame
    auto const tool_pose = world_to_robot * state.getGlobalLinkTransform(tool_name);
    return tool_pose;
  };
  PoseSequence tool_poses;
  std::transform(tool_names.cbegin(), tool_names.cend(), std::back_inserter(tool_poses), get_tool_pose);
  return tool_poses;
}

double compute_max_dist(JacobianWaypointCommand waypoint_command, PoseSequence start_tool_transforms) {
  std::vector<PointSequence> tool_paths;
  double max_dist = 0.0;
  for (auto gripper_idx = 0u; gripper_idx < waypoint_command.tools_waypoint.tools.size(); ++gripper_idx) {
    auto const &start_transform = start_tool_transforms[gripper_idx];
    auto const &target_position = waypoint_command.tools_waypoint.tools[gripper_idx].point;
    auto const dist = (target_position - start_transform.translation()).norm();
    max_dist = std::max(max_dist, dist);
  }
  return max_dist;
}

std::vector<PointSequence> interpolate_tools_waypoint(JacobianWaypointCommand waypoint_command,
                                                      PoseSequence start_tool_transforms, std::size_t const steps) {
  std::vector<PointSequence> tool_paths;
  for (auto gripper_idx = 0u; gripper_idx < waypoint_command.tools_waypoint.tools.size(); ++gripper_idx) {
    auto const start_transform = start_tool_transforms[gripper_idx];
    auto const target_position = waypoint_command.tools_waypoint.tools[gripper_idx].point;
    tool_paths.emplace_back(interpolate(start_transform.translation(), target_position, steps));
  }
  return tool_paths;
}

JacobianTrajectoryCommand make_traj_command_from_python_inputs(
    planning_scene::PlanningScenePtr planning_scene, std::string group_name, std::vector<std::string> tool_names,
    std::vector<PointSequence> grippers, robot_state::RobotState start_state,
    std::vector<Eigen::Vector4d> preferred_tool_orientations, double const max_velocity_scaling_factor,
    double const max_acceleration_scaling_factor) {
  std::vector<const robot_model::AttachedBody *> bodies;
  start_state.getAttachedBodies(bodies);
  ROS_WARN_STREAM_COND_NAMED(bodies.empty(), LOGGER_NAME, "No attached collision objects!");

  planning_scene->setCurrentState(start_state);

  // TODO: this should be an argument
  auto const world_to_robot = Pose::Identity();
  JacobianWaypointsCommand waypoints_command{
      JacobianContext{planning_scene, world_to_robot, group_name, tool_names}, {}, {}, start_state};
  JacobianTrajectoryCommand traj_command{waypoints_command, max_velocity_scaling_factor,
                                         max_acceleration_scaling_factor};
  std::transform(
      preferred_tool_orientations.cbegin(), preferred_tool_orientations.cend(),
      std::back_inserter(traj_command.waypoints_command.preferred_tool_orientations),
      [](auto const &py_preferred_tool_orientation) { return Eigen::Quaterniond{py_preferred_tool_orientation}; });
  std::transform(
      grippers.cbegin(), grippers.cend(), std::back_inserter(traj_command.waypoints_command.tools_waypoints.tools),
      [](auto const &py_tool_waypoints) {
        ToolWaypoints tool_waypoints;
        std::transform(py_tool_waypoints.cbegin(), py_tool_waypoints.cend(), std::back_inserter(tool_waypoints.points),
                       [](auto const &py_tool_waypoint) { return ToolWaypoint{py_tool_waypoint}; });
        return tool_waypoints;
      });
  return traj_command;
}

JacobianFollower::JacobianFollower(std::string const robot_namespace, std::string const robot_description,
                                   double const translation_step_size, bool const minimize_rotation,
                                   bool const collision_check, bool const visualize)
    : model_loader_(std::make_shared<robot_model_loader::RobotModelLoader>(robot_description)),
      model_(model_loader_->getModel()),
      scene_monitor_(std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(model_loader_)),
      visual_tools_("robot_root", ros::names::append(robot_namespace, "/moveit_visual_markers"), model_),
      tf_listener_(tf_buffer_),
      world_frame_("robot_root"),
      robot_frame_(model_->getRootLinkName()),
      translation_step_size_(translation_step_size),
      minimize_rotation_(minimize_rotation),
      visualize_(visualize),
      robot_namespace_(robot_namespace) {
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM_NAMED(LOGGER_NAME,
                           "You must call ros::init before using JacobianFollower. "
                               << "If you're calling this from python, use arc_utilities.ros_init.rospy_and_cpp_init");
  }
  nh_ = std::make_shared<ros::NodeHandle>();

  planning_pipeline_ = std::make_unique<planning_pipeline::PlanningPipeline>(
      model_, *nh_, "/hdt_michigan/move_group/planning_plugin", "/hdt_michigan/move_group/request_adapters");

  auto const interpolated_points_topic = ros::names::append(robot_namespace, "jacobian_waypoint_interpolated");
  vis_pub_ = std::make_shared<ros::Publisher>(
      nh_->advertise<visualization_msgs::MarkerArray>(interpolated_points_topic, 10, true));

  auto const display_planned_path_topic = ros::names::append(robot_namespace, "jacobian_follower_planned_traj");
  visual_tools_.loadTrajectoryPub(display_planned_path_topic, false);
  auto const display_robot_state_topic = ros::names::append(robot_namespace, "jacobian_follower_robot_state");
  visual_tools_.loadRobotStatePub(display_robot_state_topic, false);

  if (collision_check) {
    constraint_fun_ = [&](planning_scene::PlanningScenePtr planning_scene, robot_state::RobotState const &state) {
      return checkCollision(planning_scene, state).collision;
    };
  }
}

bool JacobianFollower::connect_to_psm() {
  ROS_INFO_STREAM_NAMED(LOGGER_NAME, "Connecting to Planning Scene Monitor");
  auto const scene_topic = ros::names::append(robot_namespace_, "move_group/monitored_planning_scene");
  scene_monitor_->startSceneMonitor(scene_topic);

  auto const service_name = ros::names::append(robot_namespace_, "get_planning_scene");
  return scene_monitor_->requestPlanningSceneState(service_name);
}

void JacobianFollower::debugLogState(const std::string prefix, robot_state::RobotState const &state) {
  std::stringstream ss;
  state.printStatePositions(ss);
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".joint_state", prefix << ss.str());
}

PlanResultMsg JacobianFollower::plan(std::string const &group_name, std::vector<std::string> const &tool_names,
                                     std::vector<Eigen::Vector4d> const &preferred_tool_orientations,
                                     std::vector<PointSequence> const &grippers,
                                     double const max_velocity_scaling_factor,
                                     double const max_acceleration_scaling_factor) {
  scene_monitor_->lockSceneRead();
  auto planning_scene = planning_scene::PlanningScene::clone(scene_monitor_->getPlanningScene());
  scene_monitor_->unlockSceneRead();

  auto const &start_state = planning_scene->getCurrentState();
  debugLogState("looked up start state: ", start_state);

  auto traj_command = make_traj_command_from_python_inputs(
      planning_scene, group_name, tool_names, grippers, start_state, preferred_tool_orientations,
      max_velocity_scaling_factor, max_acceleration_scaling_factor);
  auto const plan_result = plan(traj_command);
  moveit_msgs::RobotTrajectory plan_msg;
  plan_result.first.getRobotTrajectoryMsg(plan_msg);
  plan_msg.joint_trajectory.header.frame_id = "robot_root";
  return std::make_pair(plan_msg, plan_result.second);
}

PlanResultMsg JacobianFollower::plan(std::string const &group_name, std::vector<std::string> const &tool_names,
                                     std::vector<Eigen::Vector4d> const &preferred_tool_orientations,
                                     moveit_msgs::RobotState const &start_state_msg,
                                     std::vector<PointSequence> const &grippers,
                                     double const max_velocity_scaling_factor,
                                     double const max_acceleration_scaling_factor) {
  scene_monitor_->lockSceneRead();
  auto planning_scene = planning_scene::PlanningScene::clone(scene_monitor_->getPlanningScene());
  scene_monitor_->unlockSceneRead();

  robot_state::RobotState robot_start_state(model_);
  robotStateMsgToRobotState(start_state_msg, robot_start_state);

  auto traj_command = make_traj_command_from_python_inputs(
      planning_scene, group_name, tool_names, grippers, robot_start_state, preferred_tool_orientations,
      max_velocity_scaling_factor, max_acceleration_scaling_factor);
  auto const plan_result = plan(traj_command);
  moveit_msgs::RobotTrajectory plan_msg;
  plan_result.first.getRobotTrajectoryMsg(plan_msg);
  plan_msg.joint_trajectory.header.frame_id = "robot_root";
  return std::make_pair(plan_msg, plan_result.second);
}

PlanResultMsg JacobianFollower::plan(std::string const &group_name, std::vector<std::string> const &tool_names,
                                     std::vector<Eigen::Vector4d> const &preferred_tool_orientations,
                                     moveit_msgs::RobotState const &start_state_msg,
                                     moveit_msgs::PlanningScene const &scene_msg,
                                     std::vector<PointSequence> const &grippers,
                                     double const max_velocity_scaling_factor,
                                     double const max_acceleration_scaling_factor) {
  robot_state::RobotState robot_start_state(model_);
  robotStateMsgToRobotState(start_state_msg, robot_start_state);
  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(model_);
  planning_scene->usePlanningSceneMsg(scene_msg);

  auto traj_command = make_traj_command_from_python_inputs(
      planning_scene, group_name, tool_names, grippers, robot_start_state, preferred_tool_orientations,
      max_velocity_scaling_factor, max_acceleration_scaling_factor);
  auto const plan_result = plan(traj_command);
  moveit_msgs::RobotTrajectory plan_msg;
  plan_result.first.getRobotTrajectoryMsg(plan_msg);
  plan_msg.joint_trajectory.header.frame_id = "robot_root";
  return std::make_pair(plan_msg, plan_result.second);
}

PlanResult JacobianFollower::plan(JacobianTrajectoryCommand traj_command) {
  robot_trajectory::RobotTrajectory robot_trajectory(model_, traj_command.waypoints_command.context.group_name);

  // Validity checks
  auto const is_valid = isRequestValid(traj_command.waypoints_command);
  if (not is_valid) {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "jacobian plan inputs are not valid");
    return {robot_trajectory, true};
  }

  // NOTE: positions are assumed to be in robot_root frame
  auto const n_points = traj_command.waypoints_command.tools_waypoints.tools[0].points.size();
  auto reached_target = true;
  for (size_t waypoint_idx = 0; waypoint_idx < n_points; ++waypoint_idx) {
    auto const [traj, gripper_reached_target] = moveInWorldFrame(traj_command.waypoints_command.waypoint(waypoint_idx));
    reached_target = reached_target and gripper_reached_target;

    robot_trajectory.append(traj, 0);
  }

  if (!time_param_.computeTimeStamps(robot_trajectory, traj_command.max_velocity_scaling_factor,
                                     traj_command.max_acceleration_scaling_factor)) {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "Time parametrization for the solution path failed.");
  }

  if (visualize_ and not robot_trajectory.empty()) {
    visual_tools_.publishTrajectoryPath(robot_trajectory);
  }

  return {robot_trajectory, reached_target};
}

bool isStateValid(planning_scene::PlanningScenePtr planning_scene, moveit::core::RobotState *robot_state,
                  moveit::core::JointModelGroup const *jmg, double const *joint_positions) {
  robot_state->setJointGroupPositions(jmg, joint_positions);
  robot_state->update();  // This updates the internally stored transforms, needed before collision checking
  return planning_scene->isStateValid(*robot_state);
}

std::optional<moveit_msgs::RobotState> JacobianFollower::computeCollisionFreePointIK(
    const moveit_msgs::RobotState &default_robot_state, const std::vector<geometry_msgs::Point> &target_point,
    const std::string &group_name, const std::vector<std::string> &tip_names,
    const moveit_msgs::PlanningScene &scene_msg, IkParams const &ik_params) const {
  auto joint_model_group = model_->getJointModelGroup(group_name);
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".ik", "Tips: " << tip_names);

  bio_ik::BioIKKinematicsQueryOptions opts;
  opts.replace = true;
  opts.return_approximate_solution = false;

  for (auto tup : boost::combine(target_point, tip_names)) {
    std::string name;
    geometry_msgs::Point p;
    boost::tie(p, name) = tup;
    tf2::Vector3 position(p.x, p.y, p.z);
    opts.goals.emplace_back(std::make_unique<bio_ik::PositionGoal>(name, position));
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".ik",
                           "goal:  [" << name << "] positions " << p.x << "," << p.y << "," << p.z);
  }
  opts.goals.emplace_back(std::make_unique<bio_ik::MinimalDisplacementGoal>());

  robot_state::RobotState robot_state_ik{model_};
  robot_state::RobotState seed_robot_state_ik{model_};
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".ik", "" << default_robot_state.joint_state.name.size() << " "
                                                 << default_robot_state.joint_state.position.size());

  auto const success = moveit::core::robotStateMsgToRobotState(default_robot_state, robot_state_ik) and
                       moveit::core::robotStateMsgToRobotState(default_robot_state, seed_robot_state_ik);
  if (not success) {
    throw std::runtime_error("conversion from default_robot_state message to RobotState object failed");
  }

  robot_state_ik.update();

  // Collision checking
  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(model_);
  planning_scene->usePlanningSceneMsg(scene_msg);
  moveit::core::GroupStateValidityCallbackFn constraint_fn_boost;
  constraint_fn_boost = boost::bind(&isStateValid, planning_scene, _1, _2, _3);

  bool ok = false;
  auto attempts{0};
  for (; attempts < ik_params.max_collision_check_attempts and not ok; ++attempts) {
    robot_state_ik.setToRandomPositionsNearBy(joint_model_group, seed_robot_state_ik, ik_params.rng_dist);
    ok = robot_state_ik.setFromIK(joint_model_group,              // joints to be used for IK
                                  EigenSTL::vector_Isometry3d(),  // this isn't used, goals are described in opts
                                  std::vector<std::string>(),     // names of the end-effector links
                                  0,                              // take values from YAML
                                  constraint_fn_boost,
                                  opts  // mostly empty
    );
  }
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".ik", "ok: " << ok << ", attempts: " << attempts);

  if (ok) {
    moveit_msgs::RobotState solution_msg;
    moveit::core::robotStateToRobotStateMsg(robot_state_ik, solution_msg);
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".ik", "sln: " << solution_msg.joint_state.position);
    return solution_msg;
  }

  return {};
}

std::optional<moveit_msgs::RobotState> JacobianFollower::computeCollisionFreePoseIK(
    const moveit_msgs::RobotState &default_robot_state, const std::vector<geometry_msgs::Pose> &target_pose,
    const std::string &group_name, const std::vector<std::string> &tip_names,
    const moveit_msgs::PlanningScene &scene_msg, IkParams const &ik_params) const {
  auto joint_model_group = model_->getJointModelGroup(group_name);
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".ik", "Tips: " << tip_names);

  kinematics::KinematicsQueryOptions opts;

  robot_state::RobotState robot_state_ik(model_);
  auto const success = moveit::core::robotStateMsgToRobotState(default_robot_state, robot_state_ik);
  if (not success) {
    throw std::runtime_error("conversion from default_robot_state message to RobotState object failed");
  }
  robot_state_ik.update();

  auto const tip_transforms = EigenHelpersConversions::VectorGeometryPoseToVectorIsometry3d(target_pose);

  // Collision checking
  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(model_);
  planning_scene->usePlanningSceneMsg(scene_msg);
  moveit::core::GroupStateValidityCallbackFn constraint_fn_boost;
  constraint_fn_boost = boost::bind(&isStateValid, planning_scene, _1, _2, _3);

  bool ok = false;
  auto attempts{0};
  for (; attempts < ik_params.max_collision_check_attempts and not ok; ++attempts) {
    robot_state_ik.setToRandomPositions(joint_model_group);
    ok = robot_state_ik.setFromIK(joint_model_group, tip_transforms, tip_names, 0.0, constraint_fn_boost, opts);
  }
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".ik", "ok? " << ok << " attempts " << attempts);

  if (ok) {
    moveit_msgs::RobotState solution_msg;
    moveit::core::robotStateToRobotStateMsg(robot_state_ik, solution_msg);
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".ik", "sln: " << solution_msg.joint_state.position);
    return solution_msg;
  }
  return {};
}

std::vector<std::vector<double>> JacobianFollower::compute_IK_solutions(geometry_msgs::Pose target_pose,
                                                                        const std::string &group_name) const {
  auto const jmg = model_->getJointModelGroup(group_name);
  const kinematics::KinematicsBaseConstPtr &solver = jmg->getSolverInstance();

  Eigen::Isometry3d solverTrobot = Eigen::Isometry3d::Identity();

  auto kinematic_state = std::make_shared<robot_state::RobotState>(model_);
  kinematic_state->setToIKSolverFrame(solverTrobot, solver);

  // Convert to solver frame
  Eigen::Affine3d ee_target_pose;
  tf::poseMsgToEigen(target_pose, ee_target_pose);

  Eigen::Affine3d pt_solver = solverTrobot * ee_target_pose;

  std::vector<geometry_msgs::Pose> target_poses;
  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(pt_solver, pose);
  target_poses.push_back(pose);

  // TODO: Add seed as optional input
  std::vector<double> seed(jmg->getActiveJointModelNames().size());
  std::vector<std::vector<double>> solutions;
  kinematics::KinematicsResult result{};
  kinematics::KinematicsQueryOptions options;
  options.discretization_method = kinematics::DiscretizationMethod::ALL_DISCRETIZED;

  solver->getPositionIK(target_poses, seed, solutions, result, options);
  return solutions;
}

geometry_msgs::PoseStamped JacobianFollower::computeGroupFK(const moveit_msgs::RobotState &robot_state_msg,
                                                            const std::string &group_name) const {
  robot_state::RobotState state(model_);
  robotStateMsgToRobotState(robot_state_msg, state);

  if (not model_->hasJointModelGroup(group_name)) {
    throw std::runtime_error("Model has no group " + group_name);
  }

  auto jmg = state.getJointModelGroup(group_name);
  const auto &ee_name = jmg->getLinkModelNames().back();
  const auto &end_effector_state = state.getGlobalLinkTransform(ee_name);

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = robot_frame_;
  pose_stamped.header.stamp = ros::Time::now();
  tf::poseEigenToMsg(end_effector_state, pose_stamped.pose);
  return pose_stamped;
}

geometry_msgs::PoseStamped JacobianFollower::computeGroupFK(const std::vector<double> &joint_positions,
                                                            const std::vector<std::string> &joint_names,
                                                            const std::string &group_name) const {
  moveit_msgs::RobotState robot_state_msg;
  robot_state_msg.joint_state.position = joint_positions;
  robot_state_msg.joint_state.name = joint_names;
  return computeGroupFK(robot_state_msg, group_name);
}

geometry_msgs::PoseStamped JacobianFollower::computeFK(const moveit_msgs::RobotState &robot_state_msg,
                                                       const std::string &link_name) const {
  robot_state::RobotState state(model_);
  robotStateMsgToRobotState(robot_state_msg, state);

  if (not model_->hasLinkModel(link_name)) {
    throw std::runtime_error("Model has no link " + link_name);
  }

  const auto &end_effector_state = state.getGlobalLinkTransform(link_name);

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = robot_frame_;
  pose_stamped.header.stamp = ros::Time::now();
  tf::poseEigenToMsg(end_effector_state, pose_stamped.pose);
  return pose_stamped;
}

geometry_msgs::PoseStamped JacobianFollower::computeFK(const std::vector<double> &joint_positions,
                                                       const std::vector<std::string> &joint_names,
                                                       const std::string &link_name) const {
  moveit_msgs::RobotState robot_state_msg;
  robot_state_msg.joint_state.position = joint_positions;
  robot_state_msg.joint_state.name = joint_names;
  return computeFK(robot_state_msg, link_name);
}

bool JacobianFollower::isRequestValid(JacobianWaypointsCommand waypoints_command) const {
  if (not model_->hasJointModelGroup(waypoints_command.context.group_name)) {
    ROS_WARN_STREAM_NAMED(LOGGER_NAME, "No group " << waypoints_command.context.group_name);
    return false;
  }
  for (auto const &tool_name : waypoints_command.context.tool_names) {
    if (not model_->hasLinkModel(tool_name)) {
      ROS_WARN_STREAM("No link " << tool_name);
      return false;
    }
  }
  if (waypoints_command.tools_waypoints.tools.empty()) {
    ROS_WARN_STREAM("tools paths is empty");
    return false;
  }
  auto const first_gripper_n_points = waypoints_command.tools_waypoints.tools[0].points.size();
  for (auto const &[tool_idx, tool_path] : enumerate(waypoints_command.tools_waypoints.tools)) {
    if (tool_path.points.size() != first_gripper_n_points) {
      ROS_WARN_STREAM("mismatching number of points for different grippers. Gripper 0 has "
                      << first_gripper_n_points << " points but tool " << tool_idx << " has " << tool_path.points.size()
                      << " points.");
      return false;
    }
  }
  return true;
}

PlanResult JacobianFollower::moveInWorldFrame(JacobianWaypointCommand waypoint_command) {
  auto const *const jmg = model_->getJointModelGroup(waypoint_command.context.group_name);

  auto const start_tool_transforms = getToolTransforms(
      waypoint_command.context.world_to_robot, waypoint_command.context.tool_names, waypoint_command.start_state);
  auto const num_ees = start_tool_transforms.size();

  auto const max_dist = compute_max_dist(waypoint_command, start_tool_transforms);
  auto const steps = static_cast<std::size_t>(std::ceil(max_dist / translation_step_size_)) + 1;
  auto const tools_waypoint_interpolated = interpolate_tools_waypoint(waypoint_command, start_tool_transforms, steps);

  // visualize interpolated path
  if (visualize_) {
    visualization_msgs::MarkerArray msg;
    msg.markers.resize(num_ees);

    auto const stamp = ros::Time::now();
    for (auto tool_idx = 0ul; tool_idx < num_ees; ++tool_idx) {
      auto &path = tools_waypoint_interpolated[tool_idx];
      auto &m = msg.markers[tool_idx];
      m.ns = waypoint_command.context.tool_names[tool_idx] + "_interpolation_path";
      m.id = 1;
      m.header.frame_id = world_frame_;
      m.header.stamp = stamp;
      m.action = m.ADD;
      m.type = m.POINTS;
      m.points.resize(steps);
      m.scale.x = 0.004;
      m.scale.y = 0.004;

      m.colors.resize(steps);
      auto const start_color = ColorBuilder::MakeFromFloatColors(0, 1, 0, 1);
      auto const end_color = ColorBuilder::MakeFromFloatColors(1, 1, 0, 1);

      for (std::size_t step_idx = 0; step_idx < steps; ++step_idx) {
        m.points[step_idx] = ConvertTo<gm::Point>(path[step_idx]);
        auto const ratio = static_cast<float>(step_idx) / static_cast<float>(std::max<std::size_t>(steps - 1, 1));
        m.colors[step_idx] = arc_helpers::InterpolateColor(start_color, end_color, ratio);
      }
    }

    vis_pub_->publish(msg);
  }

  waypoint_command.context.planning_scene->setCurrentState(waypoint_command.start_state);
  auto const traj = jacobianPath3d(waypoint_command.context.planning_scene, waypoint_command.context.world_to_robot,
                                   jmg, waypoint_command.context.tool_names,
                                   waypoint_command.preferred_tool_orientations, tools_waypoint_interpolated);
  auto const n_waypoints = tools_waypoint_interpolated[0].size();  // just pick gripper 0, they are all the same
  // NOTE: if the result of jacobianPath3d has the same number of waypoints as the input (tools_waypoint_interpolated)
  // that means it reached the final position
  auto const reached_target = traj.getWayPointCount() == (n_waypoints - 1);

  // Debugging - visualize JacobianIK result tip
  if (visualize_ and !traj.empty()) {
    visualization_msgs::MarkerArray msg;
    msg.markers.resize(num_ees);

    auto const stamp = ros::Time::now();
    for (auto tool_idx = 0ul; tool_idx < num_ees; ++tool_idx) {
      auto &m = msg.markers[tool_idx];
      m.ns = waypoint_command.context.tool_names[tool_idx] + "_ik_result";
      m.id = 1;
      m.header.frame_id = world_frame_;
      m.header.stamp = stamp;
      m.action = m.ADD;
      m.type = m.POINTS;
      m.points.resize(traj.getWayPointCount());
      m.scale.x = 0.01;
      m.scale.y = 0.01;
      m.colors.resize(traj.getWayPointCount());
    }

    auto const start_color = ColorBuilder::MakeFromFloatColors(0, 0, 1, 1);
    auto const end_color = ColorBuilder::MakeFromFloatColors(1, 0, 1, 1);
    for (size_t step_idx = 0; step_idx < traj.getWayPointCount(); ++step_idx) {
      auto const &state = traj.getWayPoint(step_idx);
      auto const tool_poses =
          getToolTransforms(waypoint_command.context.world_to_robot, waypoint_command.context.tool_names, state);
      for (auto tool_idx = 0ul; tool_idx < num_ees; ++tool_idx) {
        auto &m = msg.markers[tool_idx];
        m.points[step_idx] = ConvertTo<gm::Point>(Eigen::Vector3d(tool_poses[tool_idx].translation()));
        auto const ratio =
            static_cast<float>(step_idx) / static_cast<float>(std::max((traj.getWayPointCount() - 1), 1ul));
        m.colors[step_idx] = arc_helpers::InterpolateColor(start_color, end_color, ratio);
      }
    }
    vis_pub_->publish(msg);
  }

  return {traj, reached_target};
}

robot_trajectory::RobotTrajectory JacobianFollower::jacobianPath3d(
    planning_scene::PlanningScenePtr planning_scene, Pose const &world_to_robot,
    moveit::core::JointModelGroup const *const jmg, std::vector<std::string> const &tool_names,
    EigenHelpers::VectorQuaterniond const &preferred_tool_orientations,
    std::vector<PointSequence> const &tools_waypoint_interpolated) {
  auto const num_ees = tool_names.size();
  auto const steps = tools_waypoint_interpolated[0].size();

  auto const &start_state = planning_scene->getCurrentState();
  auto const robot_to_world = world_to_robot.inverse(Eigen::Isometry);
  auto const start_tool_transforms = getToolTransforms(world_to_robot, tool_names, start_state);

  // Initialize the command with the current state for the first target point
  robot_trajectory::RobotTrajectory trajectory{model_, jmg};

  // Iteratively follow the Jacobian to each other point in the path
  ROS_DEBUG_NAMED(LOGGER_NAME, "Following Jacobian along path for group %s", jmg->getName().c_str());
  for (auto step_idx = 1ul; step_idx < steps; ++step_idx) {
    // Extract the goal positions and orientations for each tool in robot frame
    PoseSequence robotTtargets(num_ees);
    for (auto ee_idx = 0ul; ee_idx < num_ees; ++ee_idx) {
      robotTtargets[ee_idx].translation() = robot_to_world * tools_waypoint_interpolated[ee_idx][step_idx];
      robotTtargets[ee_idx].linear() = preferred_tool_orientations[ee_idx].toRotationMatrix();
    }

    // Note that jacobianIK is assumed to have modified the state in the planning scene
    const auto iksoln = jacobianIK(planning_scene, world_to_robot, jmg, tool_names, robotTtargets, constraint_fun_);
    if (!iksoln) {
      ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "IK Stalled at idx " << step_idx << ", returning early");
      break;
    }
    auto const state = planning_scene->getCurrentState();
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "step_idx " << step_idx);
    std::stringstream tool_pos_ss;
    for (auto ee_idx = 0ul; ee_idx < num_ees; ++ee_idx) {
      auto ee_pos = robotTtargets[ee_idx].translation();
      tool_pos_ss << ee_pos(0) << "," << ee_pos(1) << "," << ee_pos(2) << " ";
    }
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".tool_positions", "tool positions " << tool_pos_ss.str());
    debugLogState("jacobianIK: ", state);
    trajectory.addSuffixWayPoint(state, 0);
  }

  ROS_DEBUG_NAMED(LOGGER_NAME, "Jacobian IK path has %lu points out of a requested %lu", trajectory.getWayPointCount(),
                  steps - 1);
  return trajectory;
}

collision_detection::CollisionResult JacobianFollower::checkCollision(planning_scene::PlanningScenePtr planning_scene,
                                                                      robot_state::RobotState const &state) const {
  collision_detection::CollisionRequest collisionRequest;
  collisionRequest.contacts = true;
  collisionRequest.max_contacts = 1;
  collisionRequest.max_contacts_per_pair = 1;
  collision_detection::CollisionResult collisionResult;
  planning_scene->checkCollision(collisionRequest, collisionResult, state);
  ROS_DEBUG_STREAM_ONCE_NAMED(LOGGER_NAME + ".check_collision", "Checking collision...");
  if (collisionResult.collision) {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".check_collision", "Collision Result: " << collisionResult);
    std::stringstream ss;
    planning_scene->getAllowedCollisionMatrix().print(ss);
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".check_collision", "\n" << ss.str());

    std::vector<moveit_msgs::AttachedCollisionObject> msgs;
    planning_scene->getAttachedCollisionObjectMsgs(msgs);
    for (auto const &m : msgs) {
      ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".check_collision", "Touch Links: " << m.link_name << " " << m.touch_links);
    }

    if (visualize_) {
      //      visual_tools_.publishContactPoints(collisionResult.contacts, planning_scene.get());
      visual_tools_.publishRobotState(planning_scene->getCurrentState());
    }
  }

  return collisionResult;
}

// Note that robotTtargets is the target points for the tools, measured in robot frame
bool JacobianFollower::jacobianIK(planning_scene::PlanningScenePtr planning_scene, Pose const &world_to_robot,
                                  moveit::core::JointModelGroup const *const jmg,
                                  std::vector<std::string> const &tool_names, PoseSequence const &robotTtargets,
                                  ConstraintFn const &constraint_fn) {
  constexpr bool bPRINT = false;

  auto const robot_to_world = world_to_robot.inverse(Eigen::Isometry);

  auto const num_ees = static_cast<long>(tool_names.size());
  robot_state::RobotState &state = planning_scene->getCurrentStateNonConst();
  Eigen::VectorXd currConfig;
  state.copyJointGroupPositions(jmg, currConfig);
  Eigen::VectorXd const startConfig = currConfig;
  Eigen::VectorXd prevConfig = currConfig;
  const int ndof = static_cast<int>(startConfig.rows());

  // TODO: correctly handle DOF that have no limits throughout
  //       as well as DOF that are not in R^N, i.e.; continuous revolute, spherical etc.
  // TODO: arm->getVariableLimits()
  Eigen::VectorXd lowerLimit(ndof);
  Eigen::VectorXd upperLimit(ndof);
  {
    auto const &joints = jmg->getJointModels();
    int nextDofIdx = 0;
    for (auto joint : joints) {
      auto const &names = joint->getVariableNames();
      auto const &bounds = joint->getVariableBounds();

      for (size_t var_idx = 0; var_idx < names.size(); ++var_idx) {
        if (bounds[var_idx].position_bounded_) {
          lowerLimit[nextDofIdx] = bounds[var_idx].min_position_;
          upperLimit[nextDofIdx] = bounds[var_idx].max_position_;
        } else {
          lowerLimit[nextDofIdx] = std::numeric_limits<double>::lowest() / 4.0;
          upperLimit[nextDofIdx] = std::numeric_limits<double>::max() / 4.0;
        }
        ++nextDofIdx;
      }
    }
    if (bPRINT) {
      std::cerr << "lowerLimit:  " << lowerLimit.transpose() << "\n"
                << "upperLimit:  " << upperLimit.transpose() << "\n"
                << "startConfig: " << startConfig.transpose() << "\n\n";
    }
  }

  // Configuration variables (eventually input parameters)
  constexpr auto accuracyThreshold = 0.001;
  constexpr auto movementLimit = std::numeric_limits<double>::max();
  // setting this to true will make the algorithm attempt to move joints
  // that are at joint limits at every iteration back away from the limit
  constexpr bool clearBadJoints = true;
  const auto dampingThreshold = EigenHelpers::SuggestedRcond();
  const auto damping = EigenHelpers::SuggestedRcond();

  VecArrayXb goodJoints = VecArrayXb::Ones(ndof);
  const int maxItr = 200;
  const double maxStepSize = 0.1;
  double stepSize = maxStepSize;
  double prevError = 1000000.0;

  for (int itr = 0; itr < maxItr; itr++) {
    auto const robotTcurr = Transform(robot_to_world, getToolTransforms(world_to_robot, tool_names, state));
    auto const poseError = calcPoseError(robotTcurr, robotTtargets);
    auto const [posErrorVec, rotErrorVec] = calcVecError(poseError);
    auto currPositionError = posErrorVec.norm();
    auto currRotationError = rotErrorVec.norm();
    if (bPRINT) {
      std::cerr << std::endl << "----------- Start of for loop ---------------\n";
      std::cerr << "config = [" << currConfig.transpose() << "]';\n";

      Eigen::MatrixXd matrix_output =
          std::numeric_limits<double>::infinity() * Eigen::MatrixXd::Ones(14, 5 * num_ees - 1);
      for (auto idx = 0l; idx < num_ees; ++idx) {
        matrix_output.block<4, 4>(0, 5 * idx) = robotTcurr[static_cast<unsigned long>(idx)].matrix();
      }
      for (auto idx = 0l; idx < num_ees; ++idx) {
        matrix_output.block<4, 4>(5, 5 * idx) = robotTtargets[static_cast<unsigned long>(idx)].matrix();
      }
      for (auto idx = 0l; idx < num_ees; ++idx) {
        matrix_output.block<4, 4>(10, 5 * idx) = poseError[static_cast<unsigned long>(idx)].matrix();
      }
      std::cerr << "         ee idx ----->\n"
                << " current \n"
                << " target  \n"
                << " error   \n";
      std::cerr << "matrix_ouptut = [\n" << matrix_output << "\n];\n";
      std::cerr << "posErrorVec = [" << posErrorVec.transpose() << "]';\n"
                << "rotErrorVec = [" << rotErrorVec.transpose() << "]';\n"
                << "currPositionError: " << currPositionError << std::endl
                << "currRotationError: " << currRotationError << std::endl;
    }
    // Check if we've reached our target
    if (currPositionError < accuracyThreshold) {
      if (bPRINT) {
        std::cerr << "Projection successful itr: " << itr << ": currPositionError: " << currPositionError << "\n";
      }
      return true;
    }

    // stepSize logic and reporting
    {
      // Take smaller steps if error increases
      // if ((currPositionError >= prevError) || (prevError - currPositionError < accuracyThreshold / 10))
      if (currPositionError >= prevError) {
        if (bPRINT) {
          std::cerr << "No progress, reducing stepSize: "
                    << "prevError: " << prevError << " currErrror: " << currPositionError << std::endl;
        }
        stepSize = stepSize / 2;
        currPositionError = prevError;
        currConfig = prevConfig;

        // don't let step size get too small
        if (stepSize < accuracyThreshold / 32) {
          ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "Projection stalled itr: " << itr << " "
                                                                         << "stepSize: " << stepSize << " "
                                                                         << "accuracyThreshold: " << accuracyThreshold);
          return false;
        }
      } else {
        if (bPRINT) {
          std::cerr << "Progress, resetting stepSize to max\n";
          std::cerr << "stepSize: " << stepSize << std::endl;
        }
        stepSize = maxStepSize;
      }
    }

    // Check if we've moved too far from the start config
    if (movementLimit != std::numeric_limits<double>::infinity()) {
      if ((currConfig - startConfig).norm() > movementLimit) {
        std::cerr << "Projection hit movement limit at itr " << itr << "\n";
        return false;
      }
    }

    // If we clear bad joint inds, we try to use them again every loop;
    // this makes some sense if we're using the nullspace to servo away from joint limits
    if (clearBadJoints) {
      goodJoints = VecArrayXb::Ones(ndof);
    }

    Eigen::MatrixXd const fullJacobian = getJacobianServoFrame(jmg, tool_names, state, robotTcurr);
    Eigen::MatrixXd positionJacobian(3 * num_ees, ndof);
    Eigen::MatrixXd rotationJacobian(3 * num_ees, ndof);
    for (auto idx = 0l; idx < num_ees; ++idx) {
      positionJacobian.block(3 * idx, 0, 3, ndof) = fullJacobian.block(6 * idx, 0, 3, ndof);
      rotationJacobian.block(3 * idx, 0, 3, ndof) = fullJacobian.block(6 * idx + 3, 0, 3, ndof);
    }
    if (bPRINT) {
      std::cerr << "fullJacobian = [\n" << fullJacobian << "];\n";
      std::cerr << "positionJacobian = [\n" << positionJacobian << "];\n";
      std::cerr << "rotationJacobian = [\n" << rotationJacobian << "];\n";
    }

    bool newJointAtLimit = false;
    VecArrayXb newBadJoints;
    do {
      prevConfig = currConfig;
      prevError = currPositionError;

      // Eliminate bad joint columns from the Jacobian
      ArrayXb const jacobianMask = goodJoints.replicate(1, 3 * num_ees).transpose();
      Eigen::MatrixXd const partialPositionJacobian = jacobianMask.select(positionJacobian, 0.0);
      Eigen::MatrixXd const partialRotationJacobian = jacobianMask.select(rotationJacobian, 0.0);

      // Converts the position error vector into a unit vector if the step is too large
      const auto positionMagnitude = (currPositionError > stepSize) ? stepSize / currPositionError : 1.0;
      const Eigen::VectorXd positionCorrectionStep =
          positionMagnitude *
          EigenHelpers::UnderdeterminedSolver(partialPositionJacobian, posErrorVec, dampingThreshold, damping);
      const Eigen::VectorXd drotTransEffect = rotationJacobian * positionCorrectionStep;

      const Eigen::VectorXd drotEffective = rotErrorVec - drotTransEffect;
      const double effectiveRotationError = drotEffective.norm();
      // Converts the rotation error vector into a unit vector if the step is too large
      const auto rotationMagnitude = (effectiveRotationError > stepSize) ? stepSize / effectiveRotationError : 1.0;
      const Eigen::VectorXd rotationCorrectionStep =
          rotationMagnitude *
          EigenHelpers::UnderdeterminedSolver(partialRotationJacobian, drotEffective, dampingThreshold, damping);

      // Build the nullspace constraint matrix:
      // Jpos*q = dpos
      // [0 ... 0 1 0 ... 0]*q = 0 for bad joints
      const int ndof_at_limits = ndof - goodJoints.cast<int>().sum();
      Eigen::MatrixXd nullspaceConstraintMatrix(3 * num_ees + ndof_at_limits, ndof);
      nullspaceConstraintMatrix.topRows(3 * num_ees) = positionJacobian;
      nullspaceConstraintMatrix.bottomRows(ndof_at_limits).setConstant(0);
      int nextMatrixRowIdx = 3 * (int)num_ees;
      for (int dof_idx = 0; dof_idx < ndof; ++dof_idx) {
        if (!goodJoints(dof_idx)) {
          nullspaceConstraintMatrix(nextMatrixRowIdx, dof_idx) = 1;
          ++nextMatrixRowIdx;
        }
      }

      // Project the rotation step into the nullspace of position servoing
      Eigen::VectorXd const step =
          projectRotationIntoNullspace(positionCorrectionStep, rotationCorrectionStep, nullspaceConstraintMatrix, ndof);

      // if (bPRINT)
      // {
      //   std::cerr << "\n\n";
      //   std::cerr << "fullJacobian                  = [\n" << fullJacobian << "];\n";
      //   std::cerr << "nullspaceConstraintMatrix     = [\n" << nullspaceConstraintMatrix << "];\n";
      //   std::cerr << "nullspaceConstraintMatrixPinv = [\n" << nullspaceConstraintMatrixPinv << "];\n";
      //   std::cerr << "posErrorVec        = [" << posErrorVec.transpose() << "]';\n";
      //   std::cerr << "rotErrorVec        = [" << rotErrorVec.transpose() << "]';\n";
      //   std::cerr << "drotTransEffect    = [" << drotTransEffect.transpose() << "]';\n";
      //   std::cerr << "drotEffective      = [" << drotEffective.transpose() << "]';\n";
      //   std::cerr << "positionCorrectionStep = [" << positionCorrectionStep.transpose() << "]';\n";
      //   std::cerr << "rotationCorrectionStep = [" << rotationCorrectionStep.transpose() << "]';\n";
      //   std::cerr << "nullspaceRotationStep  = [" << nullspaceRotationStep.transpose() << "]';\n";
      //   std::cerr << "step                   = [" << step.transpose() << "]';\n\n";
      // }

      // add step and check for joint limits
      newJointAtLimit = false;
      currConfig += step;
      newBadJoints =
          ((currConfig.array() < lowerLimit.array()) || (currConfig.array() > upperLimit.array())) && goodJoints;
      newJointAtLimit = newBadJoints.any();
      goodJoints = goodJoints && !newBadJoints;

      if (bPRINT) {
        std::cerr << "lowerLimit      = [" << lowerLimit.transpose() << "]';\n";
        std::cerr << "upperLimit      = [" << upperLimit.transpose() << "]';\n";
        std::cerr << "currConfig      = [" << currConfig.transpose() << "]';\n";
        std::cerr << "newBadJoints    = [" << newBadJoints.transpose() << "]';\n";
        std::cerr << "goodJoints      = [" << goodJoints.transpose() << "]';\n";
        std::cerr << "newJointAtLimit = [" << newJointAtLimit << "]';\n";
      }

      // move back to previous point if any joint limits
      if (newJointAtLimit) {
        currConfig = prevConfig;
      }
    }
    // Exit the loop if we did not reach a new joint limit
    while (newJointAtLimit);

    // Set the robot to the current estimate and update collision info. Future calls to getCurrentRobotState will
    // reflect these new values
    state.setJointGroupPositions(jmg, currConfig);
    state.update();

    if (constraint_fn) {
      auto const constraint_violated = constraint_fn(planning_scene, state);
      if (constraint_violated) {
        ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "Projection stalled at itr " << itr);
        return false;
      }
    }
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "Iteration limit reached");
  return false;
}

Eigen::VectorXd JacobianFollower::projectRotationIntoNullspace(Eigen::VectorXd positionCorrectionStep,
                                                               Eigen::VectorXd rotationCorrectionStep,
                                                               Eigen::MatrixXd const &nullspaceConstraintMatrix,
                                                               int const ndof) {
  if (minimize_rotation_) {
    Eigen::MatrixXd const nullspaceConstraintMatrixPinv =
        EigenHelpers::Pinv(nullspaceConstraintMatrix, EigenHelpers::SuggestedRcond());
    auto const I = Eigen::MatrixXd::Identity(ndof, ndof);
    Eigen::MatrixXd const nullspaceProjector = I - (nullspaceConstraintMatrixPinv * nullspaceConstraintMatrix);
    Eigen::VectorXd const nullspaceRotationStep = nullspaceProjector * rotationCorrectionStep;
    return positionCorrectionStep + nullspaceRotationStep;
  } else {
    return positionCorrectionStep;
  }
}

Eigen::MatrixXd JacobianFollower::getJacobianServoFrame(moveit::core::JointModelGroup const *const jmg,
                                                        std::vector<std::string> const &tool_names,
                                                        robot_state::RobotState const &state,
                                                        PoseSequence const &robotTservo) {
  auto const num_ees = tool_names.size();
  auto const rows = 6 * static_cast<int>(num_ees);
  auto const columns = jmg->getVariableCount();

  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(rows, columns);
  for (auto idx = 0ul; idx < num_ees; ++idx) {
    if (not model_->hasLinkModel(tool_names[idx])) {
      ROS_FATAL_STREAM_NAMED(LOGGER_NAME, "Model has no link " << tool_names[idx]);
    }
    auto const ee_link_ = model_->getLinkModel(tool_names[idx]);
    auto const block = ::getJacobianServoFrame(jmg, state, ee_link_, robotTservo[idx]);
    jacobian.block(static_cast<long>(idx * 6), 0, 6, columns) = block;
  }
  return jacobian;
}

bool JacobianFollower::check_collision(moveit_msgs::PlanningScene const &scene_msg,
                                       moveit_msgs::RobotState const &state_msg) {
  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(model_);
  planning_scene->usePlanningSceneMsg(scene_msg);

  planning_scene->setCurrentState(state_msg);
  auto const &state = planning_scene->getCurrentState();

  std::vector<moveit_msgs::AttachedCollisionObject> msgs;
  planning_scene->getAttachedCollisionObjectMsgs(msgs);
  ROS_WARN_STREAM_COND_NAMED(msgs.empty(), LOGGER_NAME, "No attached collision objects!");
  for (auto const &m : msgs) {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".check_collision", "Touch Links: " << m.link_name << " " << m.touch_links);
  }

  auto const &collision_result = checkCollision(planning_scene, state);
  return collision_result.collision;
}

PointSequence JacobianFollower::get_tool_positions(std::vector<std::string> tool_names,
                                                   moveit_msgs::RobotState const &state_msg) {
  auto const world_to_robot = Pose::Identity();
  robot_state::RobotState state(model_);
  robotStateMsgToRobotState(state_msg, state);
  auto const tool_transforms = getToolTransforms(world_to_robot, tool_names, state);
  PointSequence positions;
  auto get_translation = [](auto const &t) { return t.translation(); };
  std::transform(tool_transforms.cbegin(), tool_transforms.cend(), std::back_inserter(positions), get_translation);
  return positions;
}
Eigen::Matrix4Xd JacobianFollower::getLinkToRobotTransform(std::vector<std::string> const &joint_names,
                                                           std::vector<double> const &joint_positions,
                                                           std::string const &link_name) {
  robot_state::RobotState state(model_);
  state.setVariablePositions(joint_names, joint_positions);
  auto const &transform = state.getGlobalLinkTransform(link_name);
  return transform.matrix();
}

std::vector<Eigen::Matrix4Xd> JacobianFollower::getLinkToRobotTransforms(
    std::vector<std::string> const &joint_names, std::vector<double> const &joint_positions,
    robot_state::RobotStatePtr state, std::vector<std::string> const &link_names) const {
  validateNamesAndPositions(joint_names, joint_positions);

  state->setVariablePositions(joint_names, joint_positions);
  std::vector<Eigen::Matrix4Xd> transforms;
  auto get_transform = [&](std::string const &link_name) {
    auto const &transform = state->getGlobalLinkTransform(link_name);
    return transform.matrix();
  };
  std::transform(link_names.cbegin(), link_names.cend(), std::back_inserter(transforms), get_transform);
  return transforms;
}

std::vector<Eigen::Matrix4Xd> JacobianFollower::getLinkToRobotTransforms(
    std::vector<std::string> const &joint_names, std::vector<double> const &joint_positions,
    std::vector<std::string> const &link_names) const {
  auto state = std::make_shared<robot_state::RobotState>(model_);
  return getLinkToRobotTransforms(joint_names, joint_positions, state, link_names);
}

std::vector<std::vector<Eigen::Matrix4Xd>> JacobianFollower::batchGetLinkToRobotTransforms(
    std::vector<std::vector<std::string>> const &joint_names, std::vector<std::vector<double>> const &joint_positions,
    std::vector<std::string> const &link_names) const {
  validateNamesAndPositions(joint_names, joint_positions);

  auto state = std::make_shared<robot_state::RobotState>(model_);
  std::vector<std::vector<Eigen::Matrix4Xd>> transforms;
  auto const batch_size = joint_names.size();
  for (auto b{0u}; b < batch_size; ++b) {
    transforms.emplace_back(getLinkToRobotTransforms(joint_names[b], joint_positions[b], state, link_names));
  }
  return transforms;
}
std::vector<std::string> JacobianFollower::getLinkNames() const { return model_->getLinkModelNames(); }

bool JacobianFollower::isCollisionChecking() const { return static_cast<bool>(constraint_fun_); }

std::optional<std::vector<double>> JacobianFollower::estimatedTorques(
    moveit_msgs::RobotState const &robot_state, std::string const &group_name,
    std::optional<std::vector<geometry_msgs::Wrench>> const &in_wrenches) const {
  auto const n_joints = robot_state.joint_state.position.size();
  auto const n_links = model_->getJointModelGroup(group_name)->getLinkModels().size();

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".dynamics", "group " << group_name);
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".dynamics", "n_joints " << n_joints);
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME + ".dynamics", "n_links " << n_links);

  std::vector<geometry_msgs::Wrench> wrenches(n_links);
  if (in_wrenches) {
    wrenches = *in_wrenches;
  }
  std::vector<double> torques_out(n_joints);
  auto const positions = robot_state.joint_state.position;
  auto const velocities = robot_state.joint_state.velocity;
  auto const accelerations = std::vector<double>(n_joints, 0.0);
  geometry_msgs::Vector3 gravity;
  gravity.z = 1;
  dynamics_solver::DynamicsSolver solver{model_, group_name, gravity};
  auto const ok = solver.getTorques(positions, velocities, accelerations, wrenches, torques_out);
  if (not ok) {
    ROS_WARN_STREAM_NAMED(LOGGER_NAME + ".dynamics", "Error computing torques");
    return {};
  }

  return torques_out;
}
moveit_msgs::PlanningScene JacobianFollower::get_scene() const {
  moveit_msgs::PlanningScene msg;
  scene_monitor_->getPlanningScene()->getPlanningSceneMsg(msg);
  return msg;
}

std::tuple<Eigen::MatrixXd, bool> JacobianFollower::getJacobian(std::string const &group_name,
                                                                std::string const &link_name,
                                                                std::vector<double> const &joint_positions) {
  auto const jmg = model_->getJointModelGroup(group_name);
  robot_state::RobotState state(model_);
  state.setVariablePositions(jmg->getActiveJointModelNames(), joint_positions);

  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  auto const success = state.getJacobian(jmg, state.getLinkModel(link_name), reference_point_position, jacobian);
  return std::tuple{jacobian, success};
}

std::string JacobianFollower::getBaseLink(std::string const &group_name) const {
  auto const jmg = model_->getJointModelGroup(group_name);
  return jmg->getCommonRoot()->getParentLinkModel()->getParentLinkModel()->getName();
}
