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
      .def(py::init<double, bool>(), py::arg("translation_step_size"), py::arg("minimize_rotation"))
      .def("plan", &JacobianFollower::plan,
           py::arg("group_name"),
           py::arg("tool_names"),
           py::arg("grippers"),
           py::arg("speed"));
}