#include <pybind11/pybind11.h>
#ifdef _OPENMP
#include <omp.h>
#endif
namespace py = pybind11;

int openmp_threads_used()
{
  int count = 0;
#pragma omp parallel
  {
#pragma omp atomic
    count++;
  }
  return count;
}
int openmp_max_threads()
{
#ifdef _OPENMP
  return omp_get_max_threads();
#else
  return 1;
#endif
}

void init_openmp(py::module &m)
{
  m.def("openmp_threads_used", &openmp_threads_used,
        "Returns number of OpenMP threads actually used");
  m.def("openmp_max_threads", &openmp_max_threads,
        "Returns the max number of threads");
}