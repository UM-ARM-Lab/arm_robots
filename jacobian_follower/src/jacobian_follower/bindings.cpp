#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pyrosmsg/pyrosmsg.h>

#include <jacobian_follower/jacobian_follower.hpp>

namespace py = pybind11;

PYBIND11_MODULE(pyjacobian_follower, m) {
  py::class_<JacobianFollower>(m, "JacobianFollower")
      .def(py::init<std::string, double, bool, bool, bool>(), py::arg("robot_namespace"),
           py::arg("translation_step_size"), py::arg("minimize_rotation"), py::arg("collision_check"),
           py::arg("visualize"))
      .def("plan_from_scene_and_state",
           py::overload_cast<std::string const &, std::vector<std::string> const &,
                             std::vector<Eigen::Vector4d> const &, moveit_msgs::RobotState const &,
                             moveit_msgs::PlanningScene const &, std::vector<PointSequence> const &, double, double>(
               &JacobianFollower::plan_from_scene_and_state),
           py::arg("group_name"), py::arg("tool_names"), py::arg("preferred_tool_orientations"), py::arg("start_state"),
           py::arg("scene"),
           py::arg("grippers"),  // TODO: rename 'grippers' to 'tool_names'
           py::arg("max_velocity_scaling_factor"), py::arg("max_acceleration_scaling_factor"))
      .def(
          "plan_from_state",
          py::overload_cast<std::string const &, std::vector<std::string> const &, std::vector<Eigen::Vector4d> const &,
                            moveit_msgs::RobotState const &, std::vector<PointSequence> const &, double, double>(
              &JacobianFollower::plan_from_state),
          py::arg("group_name"), py::arg("tool_names"), py::arg("preferred_tool_orientations"), py::arg("start_state"),
          py::arg("grippers"), py::arg("max_velocity_scaling_factor"), py::arg("max_acceleration_scaling_factor"))
      .def(
          "plan",
          py::overload_cast<std::string const &, std::vector<std::string> const &, std::vector<Eigen::Vector4d> const &,
                            std::vector<PointSequence> const &, double, double>(&JacobianFollower::plan_return_msg),
          py::arg("group_name"), py::arg("tool_names"), py::arg("preferred_tool_orientations"), py::arg("grippers"),
          py::arg("max_velocity_scaling_factor"), py::arg("max_acceleration_scaling_factor"))
      .def("compute_IK_solutions", &JacobianFollower::compute_IK_solutions, py::arg("pose"),
           py::arg("joint_group_name"))
      .def("fk", &JacobianFollower::computeFK, py::arg("joint_angles"), py::arg("joint_group_name"))
      .def("check_collision", &JacobianFollower::check_collision, py::arg("scene"), py::arg("start_state"))
      .def("get_tool_positions", &JacobianFollower::get_tool_positions, py::arg("tool_names"), py::arg("state"))
      .def("connect_to_psm", &JacobianFollower::connect_to_psm)
      //
      ;
}
