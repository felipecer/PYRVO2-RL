#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // for std::vector<Vector2>
#include <pybind11/numpy.h>
#include "RVOSimulator.h"

#ifdef _OPENMP
#include <omp.h>
#endif
namespace py = pybind11;

void init_vector2(py::module &m)
{
  py::class_<RVO::Vector2>(m, "Vector2")
      .def(py::init<float, float>(), py::arg("x") = 0.0f, py::arg("y") = 0.0f)
      .def("x", &RVO::Vector2::x)
      .def("y", &RVO::Vector2::y)
      .def("__repr__",
           [](const RVO::Vector2 &v)
           {
             std::ostringstream oss;
             oss << "(" << v.x() << ", " << v.y() << ")";
             return oss.str();
           })
      .def("__add__", [](const RVO::Vector2 &a, const RVO::Vector2 &b)
           { return a + b; })
      .def("__sub__", [](const RVO::Vector2 &a, const RVO::Vector2 &b)
           { return a - b; })
      .def("__mul__", py::overload_cast<float>(&RVO::Vector2::operator*, py::const_))
      .def("__rmul__", [](float s, const RVO::Vector2 &v)
           { return s * v; });
}

void init_line(py::module &m)
{
  py::class_<RVO::Line>(m, "Line")
      .def_readwrite("point", &RVO::Line::point)
      .def_readwrite("direction", &RVO::Line::direction);
}

void init_rvo2simulator(py::module &m)
{
  py::class_<RVO::RVOSimulator>(m, "RVOSimulator")
      // constructors
      .def(py::init<>())
      .def(py::init<float, float, size_t, float, float, float, float, const RVO::Vector2 &>(),
           py::arg("timeStep"),
           py::arg("neighborDist"),
           py::arg("maxNeighbors"),
           py::arg("timeHorizon"),
           py::arg("timeHorizonObst"),
           py::arg("radius"),
           py::arg("maxSpeed"),
           py::arg("velocity") = RVO::Vector2())
      // agent management
      .def("add_agent",
           py::overload_cast<const RVO::Vector2 &>(&RVO::RVOSimulator::addAgent),
           py::arg("position"))
      .def("add_agent",
           py::overload_cast<const RVO::Vector2 &, float, size_t, float, float, float, float, const RVO::Vector2 &>(
               &RVO::RVOSimulator::addAgent),
           py::arg("position"),
           py::arg("neighborDist"),
           py::arg("maxNeighbors"),
           py::arg("timeHorizon"),
           py::arg("timeHorizonObst"),
           py::arg("radius"),
           py::arg("maxSpeed"),
           py::arg("velocity") = RVO::Vector2())
      // obstacle management
      .def("add_obstacle", &RVO::RVOSimulator::addObstacle,
           py::arg("vertices"))
      .def("process_obstacles", &RVO::RVOSimulator::processObstacles)
      // simulation step
      .def("do_step", &RVO::RVOSimulator::doStep)
      // getters: agents
      .def("get_num_agents", &RVO::RVOSimulator::getNumAgents)
      .def("get_agent_position", &RVO::RVOSimulator::getAgentPosition, py::arg("agent_id"))
      .def("get_agent_velocity", &RVO::RVOSimulator::getAgentVelocity, py::arg("agent_id"))
      .def("get_agent_pref_velocity", &RVO::RVOSimulator::getAgentPrefVelocity, py::arg("agent_id"))
      .def("get_agent_radius", &RVO::RVOSimulator::getAgentRadius, py::arg("agent_id"))
      .def("get_agent_max_speed", &RVO::RVOSimulator::getAgentMaxSpeed, py::arg("agent_id"))
      .def("get_agent_neighbor_dist", &RVO::RVOSimulator::getAgentNeighborDist, py::arg("agent_id"))
      .def("get_agent_max_neighbors", &RVO::RVOSimulator::getAgentMaxNeighbors, py::arg("agent_id"))
      .def("get_agent_num_agent_neighbors", &RVO::RVOSimulator::getAgentNumAgentNeighbors, py::arg("agent_id"))
      .def("get_agent_num_obstacle_neighbors", &RVO::RVOSimulator::getAgentNumObstacleNeighbors, py::arg("agent_id"))
      .def("get_agent_orca_line", &RVO::RVOSimulator::getAgentORCALine, py::arg("agent_id"), py::arg("line_no"))
      .def("get_agent_agent_neighbor", &RVO::RVOSimulator::getAgentAgentNeighbor, py::arg("agent_id"), py::arg("nbr_index"))
      .def("get_agent_obstacle_neighbor", &RVO::RVOSimulator::getAgentObstacleNeighbor, py::arg("agent_id"), py::arg("nbr_index"))
      // getters: global / obstacles
      .def("get_global_time", &RVO::RVOSimulator::getGlobalTime)
      .def("get_time_step", &RVO::RVOSimulator::getTimeStep)
      .def("get_num_obstacle_vertices", &RVO::RVOSimulator::getNumObstacleVertices)
      .def("get_obstacle_vertex", &RVO::RVOSimulator::getObstacleVertex, py::arg("vertex_no"))
      .def("get_next_obstacle_vertex_no", &RVO::RVOSimulator::getNextObstacleVertexNo, py::arg("vertex_no"))
      .def("get_prev_obstacle_vertex_no", &RVO::RVOSimulator::getPrevObstacleVertexNo, py::arg("vertex_no"))
      // visibility
      .def("query_visibility",
           &RVO::RVOSimulator::queryVisibility,
           py::arg("p1"), py::arg("p2"), py::arg("radius") = 0.0f)
      // setters: agent parameters
      .def("set_agent_defaults", &RVO::RVOSimulator::setAgentDefaults,
           py::arg("neighborDist"), py::arg("maxNeighbors"),
           py::arg("timeHorizon"), py::arg("timeHorizonObst"),
           py::arg("radius"), py::arg("maxSpeed"),
           py::arg("velocity") = RVO::Vector2())
      .def("set_agent_position", &RVO::RVOSimulator::setAgentPosition, py::arg("agent_id"), py::arg("pos"))
      .def("set_agent_velocity", &RVO::RVOSimulator::setAgentVelocity, py::arg("agent_id"), py::arg("vel"))
      .def("set_agent_pref_velocity", &RVO::RVOSimulator::setAgentPrefVelocity, py::arg("agent_id"), py::arg("pref_vel"))
      .def("set_agent_radius", &RVO::RVOSimulator::setAgentRadius, py::arg("agent_id"), py::arg("radius"))
      .def("set_agent_max_speed", &RVO::RVOSimulator::setAgentMaxSpeed, py::arg("agent_id"), py::arg("max_speed"))
      .def("set_agent_neighbor_dist", &RVO::RVOSimulator::setAgentNeighborDist, py::arg("agent_id"), py::arg("nbr_dist"))
      .def("set_agent_max_neighbors", &RVO::RVOSimulator::setAgentMaxNeighbors, py::arg("agent_id"), py::arg("max_nbrs"))
      .def("set_agent_time_horizon", &RVO::RVOSimulator::setAgentTimeHorizon, py::arg("agent_id"), py::arg("t_horizon"))
      .def("set_agent_time_horizon_obst", &RVO::RVOSimulator::setAgentTimeHorizonObst, py::arg("agent_id"), py::arg("t_horizon_obst"))
      .def("get_agent_time_horizon", &RVO::RVOSimulator::getAgentTimeHorizon, py::arg("agent_id"))
      .def("get_agent_time_horizon_obst", &RVO::RVOSimulator::getAgentTimeHorizonObst, py::arg("agent_id"));
}
