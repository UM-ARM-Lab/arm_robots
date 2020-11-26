#include <boost/range/combine.hpp>

#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_ros_conversions.hpp>
#include <arc_utilities/eigen_transforms.hpp>
#include <arc_utilities/enumerate.h>
#include <arc_utilities/moveit_pose_type.hpp>
#include <arc_utilities/ostream_operators.hpp>
#include <arc_utilities/moveit_ostream_operators.hpp>
#include <arc_utilities/ros_helpers.hpp>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/String.h>

#include <jacobian_follower/jacobian_follower.hpp>
#include <jacobian_follower/jacobian_utils.hpp>

namespace gm = geometry_msgs;
using ColorBuilder = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>;
using ArrayXb = Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic>;
using VecArrayXb = Eigen::Array<bool, Eigen::Dynamic, 1>;

constexpr auto const LOGGER_NAME{"JacobianFollower"};

JacobianFollower::JacobianFollower(std::string const robot_namespace, double const translation_step_size,
                                   bool const minimize_rotation)
    : model_loader_(std::make_shared<robot_model_loader::RobotModelLoader>()),
      model_(model_loader_->getModel()),
      scene_monitor_(std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(model_loader_)),
      visual_tools_("robot_root", ros::names::append(robot_namespace, "/moveit_visual_markers")),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>()),
      world_frame_("robot_root"),
      robot_frame_(model_->getRootLinkName()),
      worldTrobot(lookupTransform(*tf_buffer_, world_frame_, robot_frame_, ros::Time(0))),
      robotTworld(worldTrobot.inverse(Eigen::Isometry)),
      translation_step_size_(translation_step_size),
      minimize_rotation_(minimize_rotation)
{
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED(LOGGER_NAME, "You must call ros::init before using JacobianFollower. "
        << "If you're calling this from python, use arc_utilities.ros_init.rospy_and_cpp_init");
  }
  nh_ = std::make_shared<ros::NodeHandle>();
  auto const waypoints_topic = ros::names::append(robot_namespace, "jacobian_waypoints");
  vis_pub_ = std::make_shared<ros::Publisher>(
      nh_->advertise<visualization_msgs::MarkerArray>(waypoints_topic, 10, true));

  auto const scene_topic = ros::names::append(robot_namespace, "move_group/monitored_planning_scene");
  auto const service_name = ros::names::append(robot_namespace, "get_planning_scene");
  scene_monitor_->startSceneMonitor(scene_topic);
  scene_monitor_->requestPlanningSceneState(service_name);
}

moveit_msgs::RobotTrajectory JacobianFollower::plan(std::string const &group_name,
                                                    std::vector<std::string> const &tool_names,
                                                    std::vector<Eigen::Vector4d> const &preferred_tool_orientations,
                                                    std::vector<std::vector<Eigen::Vector3d>> const &grippers,
                                                    double const max_velocity_scaling_factor,
                                                    double const max_acceleration_scaling_factor)
{
  // Validity checks
  auto const is_valid = isRequestValid(group_name, tool_names, grippers);
  if (not is_valid)
  {
    return {};
  }

  // NOTE: positions are assumed to be in robot_root frame
  auto const n_points = grippers[0].size();
  robot_trajectory::RobotTrajectory robot_trajectory(model_, group_name);
  PointSequence target_point_sequence;
  for (size_t waypoint_idx = 0; waypoint_idx < n_points; ++waypoint_idx)
  {
    for (auto const &gripper : grippers)
    {
      target_point_sequence.emplace_back(gripper[waypoint_idx]);
    }
    EigenHelpers::VectorQuaterniond preferred_tool_orientations_q;
    std::transform(preferred_tool_orientations.cbegin(),
                   preferred_tool_orientations.cend(),
                   std::back_inserter(preferred_tool_orientations_q),
                   [](Eigen::Vector4d x) { return Eigen::Quaterniond{x}; });
    auto const traj = moveInWorldFrame(group_name, tool_names, preferred_tool_orientations_q, target_point_sequence);
    robot_trajectory.append(traj, 0);
  }

  if (!time_param_.computeTimeStamps(robot_trajectory, max_velocity_scaling_factor, max_acceleration_scaling_factor))
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "Time parametrization for the solution path failed.");
  }

  moveit_msgs::RobotTrajectory robot_trajectory_msg;
  robot_trajectory.getRobotTrajectoryMsg(robot_trajectory_msg);
  robot_trajectory_msg.joint_trajectory.header.frame_id = "robot_root";
  return robot_trajectory_msg;
}

bool JacobianFollower::isRequestValid(std::string const &group_name,
                                      std::vector<std::string> const &tool_names,
                                      std::vector<std::vector<Eigen::Vector3d>> const &grippers) const
{
  if (not model_->hasJointModelGroup(group_name))
  {
    ROS_WARN_STREAM_NAMED(LOGGER_NAME, "No group " << group_name);
    return false;
  }
  for (auto const &tool_name : tool_names)
  {
    if (not model_->hasLinkModel(tool_name))
    {
      ROS_WARN_STREAM("No link " << tool_name);
      return false;
    }
  }
  if (grippers.empty())
  {
    ROS_WARN_STREAM("No points in the request");
    return false;
  }
  auto const first_gripper_n_points = grippers[0].size();
  for (auto const &[gripper_idx, gripper] : enumerate(grippers))
  {
    if (gripper.size() != first_gripper_n_points)
    {
      ROS_WARN_STREAM("mismatching number of points for different grippers. Gripper 0 has "
                          << first_gripper_n_points
                          << " points but gripper "
                          << gripper_idx
                          << " has "
                          << gripper.size()
                          << " points.");
      return false;
    }
  }
  return true;
}

PoseSequence JacobianFollower::getToolTransforms(std::vector<std::string> const &tool_names,
                                                 robot_state::RobotState const &state) const
{
  auto get_tool_pose = [this, state](std::string const &tool_name)
  {
    auto const tool_pose = worldTrobot * state.getGlobalLinkTransform(tool_name);
    return tool_pose;
  };
  PoseSequence tool_poses;
  std::transform(tool_names.cbegin(), tool_names.cend(), std::back_inserter(tool_poses), get_tool_pose);
  return tool_poses;
}


robot_trajectory::RobotTrajectory JacobianFollower::moveInRobotFrame(std::string const &group_name,
                                                                     std::vector<std::string> const &tool_names,
                                                                     EigenHelpers::VectorQuaterniond const &preferred_tool_orientations,
                                                                     PointSequence const &target_tool_positions)
{
  return moveInWorldFrame(group_name,
                          tool_names,
                          preferred_tool_orientations,
                          Transform(worldTrobot, target_tool_positions));
}

robot_trajectory::RobotTrajectory JacobianFollower::moveInWorldFrame(std::string const &group_name,
                                                                     std::vector<std::string> const &tool_names,
                                                                     EigenHelpers::VectorQuaterniond const &preferred_tool_orientations,
                                                                     PointSequence const &target_tool_positions)
{
  auto const *const jmg = model_->getJointModelGroup(group_name);

  planning_scene_monitor::LockedPlanningSceneRW planning_scene(scene_monitor_);
  auto const &start_state = planning_scene->getCurrentState();
  auto const start_tool_transforms = getToolTransforms(tool_names, start_state);
  auto const num_ees = start_tool_transforms.size();

  double max_dist = 0.0;

  // TODO: use C++ range library when we have access to that...
  Pose start_transform;
  Eigen::Vector3d target_position;
  BOOST_FOREACH(boost::tie(start_transform, target_position),
                boost::combine(start_tool_transforms, target_tool_positions))
        {
          auto const dist = (target_position - start_transform.translation()).norm();
          max_dist = std::max(max_dist, dist);
        }

  if (max_dist < translation_step_size_)
  {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "Motion of distance " << max_dist << " requested. Ignoring");
    return {model_, group_name};
  }
  auto const steps = static_cast<std::size_t>(std::ceil(max_dist / translation_step_size_)) + 1;
  std::vector<PointSequence> tool_paths;
  BOOST_FOREACH(boost::tie(start_transform, target_position),
                boost::combine(start_tool_transforms, target_tool_positions))
        {
          tool_paths.emplace_back(interpolate(start_transform.translation(), target_position, steps));
        }

  // visualize interpolated path
  {
    visualization_msgs::MarkerArray msg;
    msg.markers.resize(num_ees);

    auto const stamp = ros::Time::now();
    for (auto tool_idx = 0ul; tool_idx < num_ees; ++tool_idx)
    {
      auto &path = tool_paths[tool_idx];
      auto &m = msg.markers[tool_idx];
      m.ns = tool_names[tool_idx] + "_interpolation_path";
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

      for (std::size_t step_idx = 0; step_idx < steps; ++step_idx)
      {
        m.points[step_idx] = ConvertTo<gm::Point>(path[step_idx]);
        auto const ratio = static_cast<float>(step_idx) / static_cast<float>(std::max<std::size_t>(steps - 1, 1));
        m.colors[step_idx] = arc_helpers::InterpolateColor(start_color, end_color, ratio);
      }
    }

    vis_pub_->publish(msg);
  }

  auto const cmd = jacobianPath3d(planning_scene, jmg, tool_names, preferred_tool_orientations, tool_paths);

  // Debugging - visualize JacobiakIK result tip
  {
    visualization_msgs::MarkerArray msg;
    msg.markers.resize(num_ees);

    auto const stamp = ros::Time::now();
    for (auto tool_idx = 0ul; tool_idx < num_ees; ++tool_idx)
    {
      auto &m = msg.markers[tool_idx];
      m.ns = tool_names[tool_idx] + "_ik_result";
      m.id = 1;
      m.header.frame_id = world_frame_;
      m.header.stamp = stamp;
      m.action = m.ADD;
      m.type = m.POINTS;
      m.points.resize(cmd.getWayPointCount());
      m.scale.x = 0.01;
      m.scale.y = 0.01;
      m.colors.resize(cmd.getWayPointCount());
    }

    auto const start_color = ColorBuilder::MakeFromFloatColors(0, 0, 1, 1);
    auto const end_color = ColorBuilder::MakeFromFloatColors(1, 0, 1, 1);
    for (size_t step_idx = 0; step_idx < cmd.getWayPointCount(); ++step_idx)
    {
      auto const &state = cmd.getWayPoint(step_idx);
      auto const tool_poses = getToolTransforms(tool_names, state);
      for (auto tool_idx = 0ul; tool_idx < num_ees; ++tool_idx)
      {
        auto &m = msg.markers[tool_idx];
        m.points[step_idx] = ConvertTo<gm::Point>(Eigen::Vector3d(tool_poses[tool_idx].translation()));
        auto const ratio =
            static_cast<float>(step_idx) / static_cast<float>(std::max((cmd.getWayPointCount() - 1), 1ul));
        m.colors[step_idx] = arc_helpers::InterpolateColor(start_color, end_color, ratio);
      }
    }
    vis_pub_->publish(msg);
  }

  return cmd;
}

robot_trajectory::RobotTrajectory
JacobianFollower::jacobianPath3d(planning_scene_monitor::LockedPlanningSceneRW &planning_scene,
                                 moveit::core::JointModelGroup const *const jmg,
                                 std::vector<std::string> const &tool_names,
                                 EigenHelpers::VectorQuaterniond const &preferred_tool_orientations,
                                 std::vector<PointSequence> const &tool_paths)
{
  auto const num_ees = tool_names.size();
  auto const steps = tool_paths[0].size();

  auto const &start_state = planning_scene->getCurrentState();
  auto const start_tool_transforms = getToolTransforms(tool_names, start_state);

  // Initialize the command with the current state for the first target point
  robot_trajectory::RobotTrajectory cmd{model_, jmg};

  // Iteratively follow the Jacobian to each other point in the path
  ROS_DEBUG_NAMED(LOGGER_NAME, "Following Jacobian along path for group %s", jmg->getName().c_str());
  for (auto step_idx = 1ul; step_idx < steps; ++step_idx)
  {
    // Extract the goal positions and orientations for each tool in robot frame
    PoseSequence robotTtargets(num_ees);
    for (auto ee_idx = 0ul; ee_idx < num_ees; ++ee_idx)
    {
      robotTtargets[ee_idx].translation() = robotTworld * tool_paths[ee_idx][step_idx];
      robotTtargets[ee_idx].linear() = preferred_tool_orientations[ee_idx].toRotationMatrix();
    }

    // Note that jacobianIK is assumed to have modified the state in the planning scene
    const auto iksoln = jacobianIK(planning_scene, jmg, tool_names, robotTtargets);
    if (!iksoln)
    {
      ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "IK Stalled at idx " << step_idx << ", returning early");
      break;
    }
    cmd.addSuffixWayPoint(planning_scene->getCurrentState(), 0);
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME,
                         "Jacobian IK path has " << cmd.getWayPointCount() << " points out of a requested " << steps);
  return cmd;
}

// Note that robotTtargets is the target points for the tools, measured in robot frame
bool JacobianFollower::jacobianIK(
    planning_scene_monitor::LockedPlanningSceneRW &planning_scene,
    moveit::core::JointModelGroup const *const jmg,
    std::vector<std::string> const &tool_names,
    PoseSequence const &robotTtargets)
{
  constexpr bool bPRINT = false;

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
    for (auto joint : joints)
    {
      auto const &names = joint->getVariableNames();
      auto const &bounds = joint->getVariableBounds();

      for (size_t var_idx = 0; var_idx < names.size(); ++var_idx)
      {
        if (bounds[var_idx].position_bounded_)
        {
          lowerLimit[nextDofIdx] = bounds[var_idx].min_position_;
          upperLimit[nextDofIdx] = bounds[var_idx].max_position_;
        } else
        {
          lowerLimit[nextDofIdx] = std::numeric_limits<double>::lowest() / 4.0;
          upperLimit[nextDofIdx] = std::numeric_limits<double>::max() / 4.0;
        }
        ++nextDofIdx;
      }
    }
    if (bPRINT)
    {
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

  collision_detection::CollisionRequest collisionRequest;
  collisionRequest.contacts = true;
  collisionRequest.max_contacts = 1;
  collisionRequest.max_contacts_per_pair = 1;
  collision_detection::CollisionResult collisionResult;
  VecArrayXb goodJoints = VecArrayXb::Ones(ndof);
  const int maxItr = 200;
  const double maxStepSize = 0.1;
  double stepSize = maxStepSize;
  double prevError = 1000000.0;

  for (int itr = 0; itr < maxItr; itr++)
  {
    auto const robotTcurr = Transform(robotTworld, getToolTransforms(tool_names, state));
    auto const poseError = calcPoseError(robotTcurr, robotTtargets);
    auto const[posErrorVec, rotErrorVec] = calcVecError(poseError);
    auto currPositionError = posErrorVec.norm();
    auto currRotationError = rotErrorVec.norm();
    if (bPRINT)
    {
      std::cerr << std::endl << "----------- Start of for loop ---------------\n";
      std::cerr << "config = [" << currConfig.transpose() << "]';\n";

      Eigen::MatrixXd matrix_output =
          std::numeric_limits<double>::infinity() * Eigen::MatrixXd::Ones(14, 5 * num_ees - 1);
      for (auto idx = 0l; idx < num_ees; ++idx)
      {
        matrix_output.block<4, 4>(0, 5 * idx) = robotTcurr[static_cast<unsigned long>(idx)].matrix();
      }
      for (auto idx = 0l; idx < num_ees; ++idx)
      {
        matrix_output.block<4, 4>(5, 5 * idx) = robotTtargets[static_cast<unsigned long>(idx)].matrix();
      }
      for (auto idx = 0l; idx < num_ees; ++idx)
      {
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
    if (currPositionError < accuracyThreshold)
    {
      if (bPRINT)
      {
        std::cerr << "Projection successful itr: " << itr << ": currPositionError: " << currPositionError << "\n";
      }
      return true;
    }

    // stepSize logic and reporting
    {
      // Take smaller steps if error increases
      // if ((currPositionError >= prevError) || (prevError - currPositionError < accuracyThreshold / 10))
      if (currPositionError >= prevError)
      {
        if (bPRINT)
        {
          std::cerr << "No progress, reducing stepSize: "
                    << "prevError: " << prevError << " currErrror: " << currPositionError << std::endl;
        }
        stepSize = stepSize / 2;
        currPositionError = prevError;
        currConfig = prevConfig;

        // don't let step size get too small
        if (stepSize < accuracyThreshold / 32)
        {
          ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "Projection stalled itr: " << itr << " "
                                                                         << "stepSize: " << stepSize << " "
                                                                         << "accuracyThreshold: " << accuracyThreshold);
          return false;
        }
      } else
      {
        if (bPRINT)
        {
          std::cerr << "Progress, resetting stepSize to max\n";
          std::cerr << "stepSize: " << stepSize << std::endl;
        }
        stepSize = maxStepSize;
      }
    }

    // Check if we've moved too far from the start config
    if (movementLimit != std::numeric_limits<double>::infinity())
    {
      if ((currConfig - startConfig).norm() > movementLimit)
      {
        std::cerr << "Projection hit movement limit at itr " << itr << "\n";
        return false;
      }
    }

    // If we clear bad joint inds, we try to use them again every loop;
    // this makes some sense if we're using the nullspace to servo away from joint limits
    if (clearBadJoints)
    {
      goodJoints = VecArrayXb::Ones(ndof);
    }

    Eigen::MatrixXd const fullJacobian = getJacobianServoFrame(jmg, tool_names, state, robotTcurr);
    Eigen::MatrixXd positionJacobian(3 * num_ees, ndof);
    Eigen::MatrixXd rotationJacobian(3 * num_ees, ndof);
    for (auto idx = 0l; idx < num_ees; ++idx)
    {
      positionJacobian.block(3 * idx, 0, 3, ndof) = fullJacobian.block(6 * idx, 0, 3, ndof);
      rotationJacobian.block(3 * idx, 0, 3, ndof) = fullJacobian.block(6 * idx + 3, 0, 3, ndof);
    }
    if (bPRINT)
    {
      std::cerr << "fullJacobian = [\n" << fullJacobian << "];\n";
      std::cerr << "positionJacobian = [\n" << positionJacobian << "];\n";
      std::cerr << "rotationJacobian = [\n" << rotationJacobian << "];\n";
    }

    bool newJointAtLimit = false;
    VecArrayXb newBadJoints;
    do
    {
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
      int nextMatrixRowIdx = 3 * (int) num_ees;
      for (int dof_idx = 0; dof_idx < ndof; ++dof_idx)
      {
        if (!goodJoints(dof_idx))
        {
          nullspaceConstraintMatrix(nextMatrixRowIdx, dof_idx) = 1;
          ++nextMatrixRowIdx;
        }
      }

      // Project the rotation step into the nullspace of position servoing
      Eigen::VectorXd const step = projectRotationIntoNullspace(positionCorrectionStep,
                                                                rotationCorrectionStep,
                                                                nullspaceConstraintMatrix,
                                                                ndof);

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

      if (bPRINT)
      {
        std::cerr << "lowerLimit      = [" << lowerLimit.transpose() << "]';\n";
        std::cerr << "upperLimit      = [" << upperLimit.transpose() << "]';\n";
        std::cerr << "currConfig      = [" << currConfig.transpose() << "]';\n";
        std::cerr << "newBadJoints    = [" << newBadJoints.transpose() << "]';\n";
        std::cerr << "goodJoints      = [" << goodJoints.transpose() << "]';\n";
        std::cerr << "newJointAtLimit = [" << newJointAtLimit << "]';\n";
      }

      // move back to previous point if any joint limits
      if (newJointAtLimit)
      {
        currConfig = prevConfig;
      }
    }
      // Exit the loop if we did not reach a new joint limit
    while (newJointAtLimit);

    // Set the robot to the current estimate and update collision info
    state.setJointGroupPositions(jmg, currConfig);
    state.update();

    planning_scene->checkCollision(collisionRequest, collisionResult, state);
    if (collisionResult.collision)
    {
      ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "Projection stalled at itr " << itr << ". " << collisionResult);
      return false;
    }
    collisionResult.clear();
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "Iteration limit reached");
  return false;
}

Eigen::VectorXd JacobianFollower::projectRotationIntoNullspace(Eigen::VectorXd positionCorrectionStep,
                                                               Eigen::VectorXd rotationCorrectionStep,
                                                               Eigen::MatrixXd const &nullspaceConstraintMatrix,
                                                               int const ndof)
{
  if (minimize_rotation_)
  {
    Eigen::MatrixXd const nullspaceConstraintMatrixPinv = EigenHelpers::Pinv(nullspaceConstraintMatrix,
                                                                             EigenHelpers::SuggestedRcond());
    auto const I = Eigen::MatrixXd::Identity(ndof, ndof);
    Eigen::MatrixXd const nullspaceProjector = I - (nullspaceConstraintMatrixPinv * nullspaceConstraintMatrix);
    Eigen::VectorXd const nullspaceRotationStep = nullspaceProjector * rotationCorrectionStep;
    return positionCorrectionStep + nullspaceRotationStep;
  } else
  {
    return positionCorrectionStep;
  }

}

Eigen::MatrixXd JacobianFollower::getJacobianServoFrame(moveit::core::JointModelGroup const *const jmg,
                                                        std::vector<std::string> const &tool_names,
                                                        robot_state::RobotState const &state,
                                                        PoseSequence const &robotTservo)
{
  auto const num_ees = tool_names.size();
  auto const rows = 6 * static_cast<int>(num_ees);
  auto const columns = jmg->getVariableCount();

  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(rows, columns);
  for (auto idx = 0ul; idx < num_ees; ++idx)
  {
    if (not model_->hasLinkModel(tool_names[idx]))
    {
      ROS_FATAL_STREAM_NAMED(LOGGER_NAME, "Model has no link " << tool_names[idx]);
    }
    auto const ee_link_ = model_->getLinkModel(tool_names[idx]);
    auto const block = ::getJacobianServoFrame(jmg, state, ee_link_, robotTservo[idx]);
    jacobian.block(static_cast<long>(idx * 6), 0, 6, columns) = block;
  }
  return jacobian;
}
