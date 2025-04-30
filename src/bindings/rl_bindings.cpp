#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "src/include/RVO2_RL_Wrapper.h"
#include <pybind11/stl.h>

using W = RL_EXTENSIONS::RVO2_RL_Wrapper;
namespace py = pybind11;

void bind_wrapper_observation(py::class_<W> &cls)
{
    cls
        .def("get_neighbors", &RL_EXTENSIONS::RVO2_RL_Wrapper::get_neighbors,
             py::arg("agent_id"),
             R"doc(
               get_neighbors(agent_id) -> numpy.ndarray of shape (maxNeighbors, 6 or 7)

               – If mode=Cartesian, mask=False: columns [pos_x,pos_y,vel_x,vel_y,pv_x,pv_y]  
               – If mode=Polar,     mask=False: columns [pos_x,pos_y,vel_mag,vel_ang,pv_mag,pv_ang]  
               – If mask=True: same as above plus an extra final column = 1.0 (real) or 0.0 (pad)
               )doc")
        .def("get_lidar",
             &W::get_lidar,
             py::arg("agent_id"),
             R"doc(
                get_lidar(agent_id) -> numpy.ndarray of shape (N,2) or (N,3)

                Each row is:
                [angle, range]                if use_obs_mask=False
                [angle, range, mask]          if use_obs_mask=True

                - angle : float, radians from +X axis
                - range : float ∈ [0,1], normalized distance
                - mask  : 1.0 = hit, 0.0 = miss (only present if use_obs_mask=True)
            )doc");
}

void bind_wrapper_goals(py::class_<W> &cls)
{
    // SoA goal setter (vector<Vector2> version)
    cls.def("set_goals_list", &RL_EXTENSIONS::RVO2_RL_Wrapper::setGoals, py::arg("goals"));

    // NumPy‐compatible getter for all goals
    cls.def("get_goals", [](const RL_EXTENSIONS::RVO2_RL_Wrapper &wrap)
          {            
            const auto &gx = wrap.getGoalsX();
            const auto &gy = wrap.getGoalsY();
            size_t n = gx.size();
            if (gy.size() != n)
                throw std::runtime_error("Mismatched goal_x and goal_y sizes");

            py::array_t<float> arr({static_cast<py::ssize_t>(n), static_cast<py::ssize_t>(2)});
            auto buf = arr.request();
            float *ptr = (float*)buf.ptr;
            for (size_t i = 0; i < n; ++i) {
                ptr[i*2 + 0] = gx[i];
                ptr[i*2 + 1] = gy[i];
            }
            return arr; });

    // Single‐goal setter/getter
    cls.def("set_goal", &RL_EXTENSIONS::RVO2_RL_Wrapper::setGoal, py::arg("agent_id"), py::arg("goal"));
    cls.def("get_goal", [](const RL_EXTENSIONS::RVO2_RL_Wrapper &wrap, size_t id)
          {
        auto v = wrap.getGoal(id);
        return py::make_tuple(v.x(), v.y()); }, py::arg("agent_id"));
}