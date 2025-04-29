// main_bindings.cpp
#include <pybind11/pybind11.h>

// Forward declarations of your init functions
void init_vector2(pybind11::module &m);
void init_line(pybind11::module &m);
void init_rvo2simulator(pybind11::module &m);
void init_openmp(pybind11::module &m);
void init_rl_extensions_goals(pybind11::module &m);
void init_rl_extensions_wrapper(pybind11::module &m);
// void init_rl_extensions_raycasting(pybind11::module &m);

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
  auto m_rl = m.def_submodule("rl", "RL-friendly extensions to RVO2");

  init_rl_extensions_wrapper(m_rl);
  init_rl_extensions_goals(m_rl);
  // init_rl_extensions_raycasting(m_rl);
}
