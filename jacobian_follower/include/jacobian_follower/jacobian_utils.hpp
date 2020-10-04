#pragma once

#include <tf2_ros/buffer.h>
#include <ros/duration.h>
#include <arc_utilities/moveit_pose_type.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

using Matrix6Xd = Eigen::Matrix<double, 6, Eigen::Dynamic>;

Pose lookupTransform(tf2_ros::Buffer const &buffer, std::string const &parent_frame,
                     std::string const &child_frame, ros::Time const &target_time = ros::Time(0),
                     ros::Duration const &timeout = ros::Duration(0));

PointSequence interpolate(Eigen::Vector3d const &from, Eigen::Vector3d const &to, std::size_t steps);

PoseSequence calcPoseError(PoseSequence const &curr, PoseSequence const &goal);

std::pair<Eigen::VectorXd, Eigen::VectorXd> calcVecError(PoseSequence const &err);

Matrix6Xd getJacobianServoFrame(moveit::core::JointModelGroup const *jmg,
                                robot_state::RobotState const &state,
                                robot_model::LinkModel const *link,
                                Pose const &robotTservo);

