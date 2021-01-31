#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pyrosmsg/pyrosmsg.h>

#include <jacobian_follower/dual_gripper_shim.hpp>

namespace py = pybind11;

PYBIND11_MODULE(pyjacobian_follower, m)
{
  py::class_<JacobianFollower>(m, "JacobianFollower")
      .def(py::init<std::string, double, bool>(),
           py::arg("robot_namespace"),
           py::arg("translation_step_size"),
           py::arg("minimize_rotation"))
      .def("plan_from_start_state", py::overload_cast<std::string const &,
               std::vector<std::string> const &,
               std::vector<Eigen::Vector4d> const &,
               moveit_msgs::RobotState const &,
               std::vector<std::vector<Eigen::Vector3d>> const &,
               double,
               double>(&JacobianFollower::plan_return_msg),
           py::arg("group_name"),
           py::arg("tool_names"),
           py::arg("preferred_tool_orientations"),
           py::arg("start_state"),
           py::arg("grippers"),
           py::arg("max_velocity_scaling_factor"),
           py::arg("max_acceleration_scaling_factor")
      )
      .def("plan", py::overload_cast<std::string const &,
               std::vector<std::string> const &,
               std::vector<Eigen::Vector4d> const &,
               std::vector<std::vector<Eigen::Vector3d>> const &,
               double,
               double>(&JacobianFollower::plan_return_msg),
           py::arg("group_name"),
           py::arg("tool_names"),
           py::arg("preferred_tool_orientations"),
           py::arg("grippers"),
           py::arg("max_velocity_scaling_factor"),
           py::arg("max_acceleration_scaling_factor")
      );
}