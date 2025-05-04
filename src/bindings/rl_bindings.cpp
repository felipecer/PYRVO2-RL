#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "src/include/RVO2_RL_Wrapper.h"
#include <pybind11/stl.h>

using W = RL_EXTENSIONS::RVO2_RL_Wrapper;
namespace py = pybind11;

void bind_wrapper_general_purpose_functions(py::class_<W> &cls)
{
  cls.def("initialize", &RL_EXTENSIONS::RVO2_RL_Wrapper::initialize,
          R"doc(
           Initialize the RVO2 wrapper. This must be called after adding agents and setting goals.
           Throws:
               RuntimeError: If simulator is not initialized
               RuntimeError: If no agents have been added 
               RuntimeError: If goals are not properly set for all agents
               RuntimeError: If LIDAR is enabled but RayCastingEngine is not initialized
           )doc");

  cls.def("add_agent",
          py::overload_cast<const RVO::Vector2 &>(&RL_EXTENSIONS::RVO2_RL_Wrapper::add_agent),
          py::arg("position"));
  cls.def("add_agent",
          py::overload_cast<const RVO::Vector2 &, float, size_t, float, float, float, float, const RVO::Vector2 &>(
              &RL_EXTENSIONS::RVO2_RL_Wrapper::add_agent),
          py::arg("position"),
          py::arg("neighborDist"),
          py::arg("maxNeighbors"),
          py::arg("timeHorizon"),
          py::arg("timeHorizonObst"),
          py::arg("radius"),
          py::arg("maxSpeed"),
          py::arg("velocity") = RVO::Vector2());

  cls.def("set_current_goals_as_initial_goals", &RL_EXTENSIONS::RVO2_RL_Wrapper::setCurrentGoalsAsInitialGoals,
          R"doc(
           Sets the current goals as the initial goals for all agents.
           )doc");

  cls.def("set_current_positions_as_initial_positions", &RL_EXTENSIONS::RVO2_RL_Wrapper::setCurrentPositionsAsInitialPositions,
          R"doc(
           Sets the current positions as the initial positions for all agents.
           )doc");

  cls.def("reset_position_and_goals_to_init", &RL_EXTENSIONS::RVO2_RL_Wrapper::resetPositionAndGoalsToInit,
          R"doc(
           Resets the positions and goals of all agents to their initial states.
           )doc");

  cls.def("get_agent_behavior", &RL_EXTENSIONS::RVO2_RL_Wrapper::getAgentBehavior,
          py::arg("agent_id"),
          R"doc(
           Returns the behavior of the specified agent.
           )doc");

  cls.def("set_agent_behavior", &RL_EXTENSIONS::RVO2_RL_Wrapper::setAgentBehavior,
          py::arg("agent_id"),
          py::arg("behavior"),
          R"doc(
           Sets the behavior of the specified agent.
           )doc");

  cls.def("initialize_agent_behaviors", &RL_EXTENSIONS::RVO2_RL_Wrapper::initializeAgentBehaviors,
          py::arg("num_agents"),
          py::arg("default_behavior") = "",
          R"doc(
           Initializes the behaviors for all agents with an optional default behavior.
           )doc");

  cls.def("get_agent_data_for_vis", [](const RL_EXTENSIONS::RVO2_RL_Wrapper &wrap)
          {
            auto vec = wrap.collectAgentsBatchData();
            py::list pylist;

            for (const auto &d : vec)
            {
              pylist.append(py::make_tuple(
                  d.id,
                  d.px, d.py,
                  py::make_tuple(d.vx, d.vy),
                  py::make_tuple(d.pvx, d.pvy),
                  d.dist_goal));
            }
            return pylist; // Python list[tuple]
          },
          R"doc(
          Return a list of tuples:

              (agent_id, x, y, (vx, vy), (pvx, pvy), dist_to_goal)

          All agents are gathered in one C++ call, eliminating per‑agent overhead.
          )doc");
}

void bind_wrapper_observation(py::class_<W> &cls)
{
  cls
      .def("get_observation_limits", &RL_EXTENSIONS::RVO2_RL_Wrapper::get_observation_bounds,
           R"doc(
            Return a dict with the following entries:
            
                - mode:       "cartesian" or "polar"
                - low:        NumPy array of per-dim lower bounds
                - high:       NumPy array of per-dim upper bounds
                - info:       list of human-readable strings describing each slice
            
            The layout matches exactly the flattened observation vector:
            [ step,
                agent_x, agent_y,
                (LIDAR: angles, ranges[, mask]),
                (neighbors: cartesian or polar),
                (neighbor mask?) ]
            )doc")
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
            )doc")
      .def("get_observation", [](W &w, int agent_id)
           {
                  namespace np = pybind11;
                  using ssize = pybind11::ssize_t;
   
                  // 1) Always zero‐placeholder step
                  float step = 0.0f;
   
                  // 2) dist_to_goal_x,y
                  const auto &dxs = w.getDistToGoalsX();
                  const auto &dys = w.getDistToGoalsY();
                  float dx = dxs.at(agent_id);
                  float dy = dys.at(agent_id);
   
                  // 3) Try LIDAR
                  np::array_t<float> lidar;
                  ssize R = 0, C = 0;
                  if (w.isUsingLidar()) {
                    lidar = w.get_lidar(agent_id);
                    auto shape = lidar.shape();
                    R = shape[0];
                    C = shape[1];
                  }
   
                  // 4) Always neighbors
                  auto nbrs = w.get_neighbors(agent_id);
                  auto nsh  = nbrs.shape();
                  ssize M   = nsh[0];
                  ssize CN  = nsh[1];
   
                  // 5) total length
                  ssize total = 1    // step
                                + 2  // dx, dy
                                + R*C
                                + M*CN;
                  np::array_t<float> out({total});
                  auto buf = out.mutable_unchecked<1>();
   
                  // 6) fill step, dx, dy
                  ssize idx = 0;
                  buf(idx++) = step;
                  buf(idx++) = dx;
                  buf(idx++) = dy;
   
                  // 7) flatten lidar if any
                  if (R > 0) {
                    auto lbuf = lidar.unchecked<2>();
                    for (ssize i = 0; i < R; ++i)
                      for (ssize j = 0; j < C; ++j)
                        buf(idx++) = lbuf(i, j);
                  }
   
                  // 8) flatten neighbors
                  auto nbuf = nbrs.unchecked<2>();
                  for (ssize i = 0; i < M; ++i)
                    for (ssize j = 0; j < CN; ++j)
                      buf(idx++) = nbuf(i, j);
   
                  return out; }, py::arg("agent_id"),
           R"doc(
                   get_observation(agent_id) -> 1D numpy array
   
                   Concatenates, in order:
                     [ step=0,
                       dist_to_goal_x, dist_to_goal_y,
                       *all lidar rows flattened (angle,range[,mask])*,
                       *all neighbor rows flattened* ]
                   Works whether or not LIDAR was enabled on this wrapper.
                )doc");
}

void bind_wrapper_goals(py::class_<W> &cls)
{
  // SoA goal setter (vector<Vector2> version)
  cls.def("set_goals", &RL_EXTENSIONS::RVO2_RL_Wrapper::setGoals, py::arg("goals"));

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

  cls.def("get_all_distances_to_goals", &RL_EXTENSIONS::RVO2_RL_Wrapper::getAllDistancesToGoals,
          py::arg("normalized") = false,
          R"doc(
           Returns a list of distances from all agents to their goals.
           If `normalized` is True, the distances are normalized against the original distances.
           )doc");

  cls.def("get_distance_to_goal", [](const RL_EXTENSIONS::RVO2_RL_Wrapper &wrap, size_t agent_id, bool normalized)
          {
    try {
        return wrap.getDistanceToGoal(agent_id, normalized);
    } catch (const std::out_of_range &e) {
        throw std::runtime_error(std::string("Out of range error in get_distance_to_goal: ") + e.what());
    } catch (const std::exception &e) {
        throw std::runtime_error(std::string("General error in get_distance_to_goal: ") + e.what());
    } }, py::arg("agent_id"), py::arg("normalized") = false, R"doc(Returns the distance from a specific agent to its goal. If `normalized` is True, the distance is normalized against the original distance.)doc");
}
