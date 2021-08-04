#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pyrosmsg/pyrosmsg.h>

#include <jacobian_follower/jacobian_follower.hpp>

namespace py = pybind11;

PYBIND11_MODULE(pyjacobian_follower, m) {
  py::class_<JacobianFollower>(m, "JacobianFollower")
      .def(py::init<std::string, std::string, double, bool, bool, bool>(), py::arg("robot_namespace"),
           py::arg("robot_description"), py::arg("translation_step_size"), py::arg("minimize_rotation"),
           py::arg("collision_check"), py::arg("visualize"))
      .def(
          "plan",
          py::overload_cast<std::string const &, std::vector<std::string> const &, std::vector<Eigen::Vector4d> const &,
                            moveit_msgs::RobotState const &, moveit_msgs::PlanningScene const &,
                            std::vector<PointSequence> const &, double, double>(&JacobianFollower::plan),
          py::arg("group_name"), py::arg("tool_names"), py::arg("preferred_tool_orientations"), py::arg("start_state"),
          py::arg("scene"), py::arg("grippers"), py::arg("max_velocity_scaling_factor"),
          py::arg("max_acceleration_scaling_factor"))
      .def(
          "plan",
          py::overload_cast<std::string const &, std::vector<std::string> const &, std::vector<Eigen::Vector4d> const &,
                            moveit_msgs::RobotState const &, std::vector<PointSequence> const &, double, double>(
              &JacobianFollower::plan),
          py::arg("group_name"), py::arg("tool_names"), py::arg("preferred_tool_orientations"), py::arg("start_state"),
          py::arg("grippers"), py::arg("max_velocity_scaling_factor"), py::arg("max_acceleration_scaling_factor"))
      .def(
          "plan",
          py::overload_cast<std::string const &, std::vector<std::string> const &, std::vector<Eigen::Vector4d> const &,
                            std::vector<PointSequence> const &, double, double>(&JacobianFollower::plan),
          py::arg("group_name"), py::arg("tool_names"), py::arg("preferred_tool_orientations"), py::arg("grippers"),
          py::arg("max_velocity_scaling_factor"), py::arg("max_acceleration_scaling_factor"))
      .def("compute_IK_solutions", &JacobianFollower::compute_IK_solutions, py::arg("pose"),
           py::arg("joint_group_name"))
      .def("compute_collision_free_pose_ik", &JacobianFollower::computeCollisionFreePoseIK,
           py::arg("default_robot_state"), py::arg("poses"), py::arg("group_name"), py::arg("tip_names"),
           py::arg("scene_name"), py::arg("ik_params"))
      .def("compute_collision_free_point_ik", &JacobianFollower::computeCollisionFreePointIK,
           py::arg("default_robot_state"), py::arg("points"), py::arg("group_name"), py::arg("tip_names"),
           py::arg("scene_name"), py::arg("ik_params"))
      .def("group_fk",
           py::overload_cast<const std::vector<double> &, const std::vector<std::string> &, const std::string &>(
               &JacobianFollower::computeGroupFK, py::const_),
           py::arg("joint_angles"), py::arg("joint_names"), py::arg("group_name"))
      .def("fk",
           py::overload_cast<const std::vector<double> &, const std::vector<std::string> &, const std::string &>(
               &JacobianFollower::computeFK, py::const_),
           py::arg("joint_angles"), py::arg("joint_names"), py::arg("link_name"))
      .def("group_fk",
           py::overload_cast<const moveit_msgs::RobotState &, const std::string &>(&JacobianFollower::computeGroupFK,
                                                                                   py::const_),
           py::arg("robot_state"), py::arg("group_name"))
      .def("fk",
           py::overload_cast<const moveit_msgs::RobotState &, const std::string &>(&JacobianFollower::computeFK,
                                                                                   py::const_),
           py::arg("robot_state"), py::arg("link_name"))
      .def("check_collision", &JacobianFollower::check_collision, py::arg("scene"), py::arg("start_state"))
      .def("get_tool_positions", &JacobianFollower::get_tool_positions, py::arg("tool_names"), py::arg("state"))
      .def("connect_to_psm", &JacobianFollower::connect_to_psm)
      .def("get_link_to_robot_transform", &JacobianFollower::getLinkToRobotTransform)
      .def("get_link_to_robot_transforms",
           py::overload_cast<std::vector<std::string> const &, std::vector<double> const &,
                             std::vector<std::string> const &>(&JacobianFollower::getLinkToRobotTransforms, py::const_))
      .def("batch_get_link_to_robot_transforms", &JacobianFollower::batchGetLinkToRobotTransforms)
      .def("get_link_names", &JacobianFollower::getLinkNames)
      .def("is_collision_checking", &JacobianFollower::isCollisionChecking)
      //
      ;
  py::class_<IkParams>(m, "IkParams")
      .def(py::init<double, int>(), py::arg("rng_dist"), py::arg("max_collision_check_attempts"))
      //
      ;
}
