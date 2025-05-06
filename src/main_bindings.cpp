// main_bindings.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "src/include/RVO2_RL_Wrapper.h"
using W = RL_EXTENSIONS::RVO2_RL_Wrapper;
using SimRef = RVO::RVOSimulator &;

// Forward declarations of your init functions
void init_vector2(pybind11::module &m);
void init_line(pybind11::module &m);
void init_rvo2simulator(pybind11::module &m);
void init_openmp(pybind11::module &m);
void bind_wrapper_observation(pybind11::class_<W> &cls);
void bind_wrapper_goals(pybind11::class_<W> &cls);
void bind_wrapper_general_purpose_functions(pybind11::class_<W> &cls);

namespace py = pybind11;

PYBIND11_MODULE(rvo2_rl, m)
{
     m.doc() = "rvo2_rl: plain RVO2 + RL-friendly extensions";

     // 1) "rvo2" submodule: the vanilla RVO2 API
     auto m_rvo = m.def_submodule("rvo2", "Pure RVO2 simulator bindings");
     init_vector2(m_rvo);
     init_line(m_rvo);
     init_rvo2simulator(m_rvo);

     // 2) "util" submodule: OpenMP & other utilities
     auto m_util = m.def_submodule("util", "Utility functions");
     init_openmp(m_util);

     // 3) "rl" submodule: your reinforcement-learning extensions
     auto m_rl = m.def_submodule("rl", "Reinforcement Learning extensions to RVO2");
     // This is the “base” binding for your wrapper class—
     // all methods hang off of this one py::class_<> object.
     py::enum_<RL_EXTENSIONS::ObsMode>(m_rl, "ObsMode")
         .value("Cartesian", RL_EXTENSIONS::ObsMode::Cartesian)
         .value("Polar", RL_EXTENSIONS::ObsMode::Polar)
         .export_values();
     auto cls = py::class_<RL_EXTENSIONS::RVO2_RL_Wrapper>(m_rl, "RVO2RLWrapper")
                    // Expose its constructor signature:
                    .def(py::init<
                             float, float, size_t, float, float, float, float,
                             const RVO::Vector2 &, RL_EXTENSIONS::ObsMode, bool, bool, size_t, float, size_t>(),
                         py::arg("time_step") = 0.25f,
                         py::arg("neighbor_dist") = 15.0f,
                         py::arg("max_neighbors") = 10,
                         py::arg("time_horizon") = 5.0f,
                         py::arg("time_horizon_obst") = 5.0f,
                         py::arg("radius") = 0.5f,
                         py::arg("max_speed") = 2.0f,
                         py::arg("velocity") = RVO::Vector2(),
                         py::arg("mode") = RL_EXTENSIONS::ObsMode::Cartesian,
                         py::arg("use_obs_mask") = false,
                         py::arg("use_lidar") = false,
                         py::arg("lidar_count") = 360,
                         py::arg("lidar_range") = 18.0f,
                         py::arg("max_step_count") = 256)
                    // You can also bind any “always‐on” methods right here:
                    .def("set_preferred_velocities",
                         &RL_EXTENSIONS::RVO2_RL_Wrapper::setPreferredVelocities)
                    .def("set_preferred_velocity",
                         &RL_EXTENSIONS::RVO2_RL_Wrapper::setPreferredVelocity, py::arg("agent_id"), py::arg("velocity"))
                    .def("get_simulator",
                         static_cast<SimRef (W::*)()>(&W::getSimulator),
                         py::return_value_policy::reference_internal,
                         "Returns the internal RVOSimulator (mutable)")
                    .def("get_step_count", &RL_EXTENSIONS::RVO2_RL_Wrapper::getStepCount)
                    .def("reset_step_count", &RL_EXTENSIONS::RVO2_RL_Wrapper::resetStepCount)
                    .def("do_step", &RL_EXTENSIONS::RVO2_RL_Wrapper::do_step);
     bind_wrapper_general_purpose_functions(cls);
     bind_wrapper_goals(cls);
     bind_wrapper_observation(cls);
     // init_rl_extensions_raycasting(m_rl);
}
