#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/eigen_ros_conversions.hpp>
#include <arc_utilities/eigen_std_conversions.hpp>
#include <jacobian_follower/jacobian_utils.hpp>
#include <arc_utilities/eigen_helpers.hpp>

Pose lookupTransform(tf2_ros::Buffer const &buffer, std::string const &parent_frame,
                            std::string const &child_frame, ros::Time const &target_time,
                            ros::Duration const &timeout)
{
  // Wait for up to timeout amount of time, then try to lookup the transform,
  // letting TF2's exception handling throw if needed
  if (!buffer.canTransform(parent_frame, child_frame, target_time, timeout))
  {
    ROS_WARN_STREAM("Unable to lookup transform between " << parent_frame << " and " << child_frame
                                                          << ". Defaulting to Identity.");
    return Pose::Identity();
  }
  auto const transform = buffer.lookupTransform(parent_frame, child_frame, target_time);
  return ConvertTo<Pose>(transform.transform);
}

PointSequence interpolate(Eigen::Vector3d const &from, Eigen::Vector3d const &to, std::size_t const steps)
{
  PointSequence sequence;
  sequence.resize(steps);
  for (std::size_t i = 0; i < steps; ++i)
  {
    const double t = static_cast<double>(i) / static_cast<double>(steps - 1);
    sequence[i] = ((1.0 - t) * from) + (t * to);
  }
  return sequence;
}

PoseSequence calcPoseError(PoseSequence const &curr, PoseSequence const &goal)
{
  assert(curr.size() == goal.size());
  auto const n = curr.size();
  PoseSequence err(n);
  for (auto idx = 0ul; idx < n; ++idx)
  {
    err[idx] = curr[idx].inverse(Eigen::Isometry) * goal[idx];
  }
  return err;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> calcVecError(PoseSequence const &err)
{
  Eigen::VectorXd posVec(err.size() * 3);
  Eigen::VectorXd rotVec(err.size() * 3);
  for (auto idx = 0ul; idx < err.size(); ++idx)
  {
    auto const twist = EigenHelpers::TwistUnhat(err[idx].matrix().log());
    posVec.segment<3>(static_cast<long>(3 * idx)) = err[idx].translation();
    rotVec.segment<3>(static_cast<long>(3 * idx)) = twist.tail<3>();
  }
  return {posVec, rotVec};
}

// See MLS Page 115-121
// https://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-complete.pdf
Matrix6Xd getJacobianServoFrame(moveit::core::JointModelGroup const *const jmg,
                                robot_state::RobotState const &state,
                                robot_model::LinkModel const *link, Pose const &robotTservo)
{
  const Pose reference_transform = robotTservo.inverse(Eigen::Isometry);
  const robot_model::JointModel *root_joint_model = jmg->getJointModels()[0];

  auto const rows = 6;
  auto const columns = jmg->getVariableCount();
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(rows, columns);

  Eigen::Vector3d joint_axis;
  Pose joint_transform;
  while (link)
  {
    const robot_model::JointModel *pjm = link->getParentJointModel();
    if (pjm->getVariableCount() > 0)
    {
      // TODO: confirm that variables map to unique joint indices
      auto const joint_index = jmg->getVariableGroupIndex(pjm->getName());
      if (pjm->getType() == robot_model::JointModel::REVOLUTE)
      {
        joint_transform = reference_transform * state.getGlobalLinkTransform(link);
        joint_axis = joint_transform.rotation() * dynamic_cast<robot_model::RevoluteJointModel const *>(pjm)->getAxis();
        jacobian.block<3, 1>(0, joint_index) = -joint_axis.cross(joint_transform.translation());
        jacobian.block<3, 1>(3, joint_index) = joint_axis;
      } else if (pjm->getType() == robot_model::JointModel::PRISMATIC)
      {
        joint_transform = reference_transform * state.getGlobalLinkTransform(link);
        joint_axis =
            joint_transform.rotation() * dynamic_cast<robot_model::PrismaticJointModel const *>(pjm)->getAxis();
        jacobian.block<3, 1>(0, joint_index) = joint_axis;
      } else if (pjm->getType() == robot_model::JointModel::PLANAR)
      {
        // This is an SE(2) joint
        joint_transform = reference_transform * state.getGlobalLinkTransform(link);
        joint_axis = joint_transform * Eigen::Vector3d::UnitX();
        jacobian.block<3, 1>(0, joint_index) = joint_axis;
        joint_axis = joint_transform * Eigen::Vector3d::UnitY();
        jacobian.block<3, 1>(0, joint_index + 1) = joint_axis;
        joint_axis = joint_transform * Eigen::Vector3d::UnitZ();
        jacobian.block<3, 1>(0, joint_index + 2) = -joint_axis.cross(joint_transform.translation());
        jacobian.block<3, 1>(3, joint_index + 2) = joint_axis;
      } else
      {
        ROS_ERROR("Unknown type of joint in Jacobian computation");
      }
    }
    // NB: this still works because we all joints that are not directly in
    // the kinematic chain have 0s in the Jacobian as they should
    if (pjm == root_joint_model)
    {
      break;
    }
    // NB: this still works because we all joints that are not directly in
    // the kinematic chain have 0s in the Jacobian as they should
    link = pjm->getParentLinkModel();
  }
  return jacobian;
}
