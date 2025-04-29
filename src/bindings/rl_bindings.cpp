#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "src/rvo2/include/Definitions.h"
#include "src/include/RVO2_RL_Wrapper.h"

namespace py = pybind11;

void init_rl_extensions_wrapper(py::module &m)
{
    py::enum_<RL_EXTENSIONS::ObsMode>(m, "ObsMode")
        .value("Cartesian", RL_EXTENSIONS::ObsMode::Cartesian)
        .value("Polar", RL_EXTENSIONS::ObsMode::Polar)
        .export_values();

    py::class_<RL_EXTENSIONS::RVO2_RL_Wrapper>(m, "RVO2RLWrapper")
        // constructor now takes an ObsMode and a bool useMask
        .def(py::init<
                 float, float, size_t, float, float, float, float,
                 const RVO::Vector2 &, RL_EXTENSIONS::ObsMode, bool>(),
             py::arg("time_step") = 0.25f,
             py::arg("neighbor_dist") = 15.0f,
             py::arg("max_neighbors") = 10,
             py::arg("time_horizon") = 5.0f,
             py::arg("time_horizon_obst") = 5.0f,
             py::arg("radius") = 0.5f,
             py::arg("max_speed") = 2.0f,
             py::arg("velocity") = RVO::Vector2(),
             py::arg("mode") = RL_EXTENSIONS::ObsMode::Cartesian,
             py::arg("use_obs_mask") = false)
        .def("set_preferred_velocities", &RL_EXTENSIONS::RVO2_RL_Wrapper::setPreferredVelocities)
        .def("get_neighbors", &RL_EXTENSIONS::RVO2_RL_Wrapper::get_neighbors,
             py::arg("agent_id"),
             R"doc(
                get_neighbors(agent_id) -> numpy.ndarray of shape (maxNeighbors, 6 or 7)

                – If mode=Cartesian, mask=False: columns [pos_x,pos_y,vel_x,vel_y,pv_x,pv_y]  
                – If mode=Polar,     mask=False: columns [pos_x,pos_y,vel_mag,vel_ang,pv_mag,pv_ang]  
                – If mask=True: same as above plus an extra final column = 1.0 (real) or 0.0 (pad)
                )doc");
}

void init_rl_extensions_goals(py::module &m)
{
    // SoA goal setter (vector<Vector2> version)
    m.def("set_goals_list", &RL_EXTENSIONS::RVO2_RL_Wrapper::setGoals, py::arg("goals"));

    // NumPy‐compatible getter for all goals
    m.def("get_goals", [](const RL_EXTENSIONS::RVO2_RL_Wrapper &wrap)
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
    m.def("set_goal", &RL_EXTENSIONS::RVO2_RL_Wrapper::setGoal, py::arg("agent_id"), py::arg("goal"));
    m.def("get_goal", [](const RL_EXTENSIONS::RVO2_RL_Wrapper &wrap, size_t id)
          {
        auto v = wrap.getGoal(id);
        return py::make_tuple(v.x(), v.y()); }, py::arg("agent_id"));
}

// void init_rl_extensions_raycasting(py::module &m)
// {
//     m.def("init_raycasting_engine", &RVO::RVOSimulator::initRaycastingEngine, py::arg("count") = 360, py::arg("length") = 18.0f,
//           R"doc(
//           Initialise (or re‑initialise) the internal RayCastingEngine.

//           Parameters
//           ----------
//           count : int, optional
//               How many evenly spaced rays to pre‑bake (default 360).

//           length : float, optional
//               World‑space length of each ray (default 18.0).

//           Calling this again replaces the previous engine instance; any memory
//           held by it is released automatically.
//          )doc");
//     m.def("get_raycasting", [](const RVO::RVOSimulator &sim, int agent_id)
//           {
//           // 1) ask the simulator to fill the hit arrays
//           std::vector<float> out_x;
//           std::vector<float> out_y;
//           sim.getRayCast(agent_id, out_x, out_y);

//           int n = out_x.size(); // fan count (360, 540, …)

//           // 2) pack into a contiguous (n*2) float buffer
//           float *buf = new float[n * 2];
//           for (std::size_t i = 0; i < n; ++i)
//           {
//                buf[2 * i] = out_x[i];
//                buf[2 * i + 1] = out_y[i];
//           }

//           // 3) create a capsule to own the buffer
//           py::capsule free_when_done(buf, [](void *p)
//                                      { delete[] reinterpret_cast<float *>(p); });

//           // 4) expose as NumPy array with shape (n, 2)
//           return py::array_t<float>(
//               /* shape   */ {n, 2},
//               /* strides */ {sizeof(float) * 2, sizeof(float)},
//               /* data    */ buf,
//               /* owner   */ free_when_done); }, py::arg("agent_id"),
//           R"doc(
//             Cast the pre‑baked ray fan for the given agent.

//             Returns a NumPy array of shape (N, 2) where N is the ray count
//             configured in initRaycastingEngine.  Each row is [x, y] of the
//             first intersection; rows contain NaN if the ray misses.
//       )doc");
//     m.def("get_raycasting_processed", [](const RVO::RVOSimulator &sim, int agent_id)
//           {
//     std::vector<float> distances;
//     std::vector<uint8_t> mask;
//     sim.getRayCastingProcessed(agent_id, distances, mask);

//     return py::make_tuple(
//         py::array_t<float>(distances.size(), distances.data()),
//         py::array_t<uint8_t>(mask.size(), mask.data())
//     ); }, py::arg("agent_id"),
//           R"doc(
//     Returns a tuple of:
//     - distances (normalized to [0, 1])
//     - mask (1 = hit, 0 = miss)

//     Useful for direct NN input.
// )doc")
// }
