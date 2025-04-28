from setuptools import setup, Extension
import pybind11
import os

# Base directories
SRC_DIR = "src"
INCLUDE_DIR = os.path.join(SRC_DIR, "include")
IMPL_DIR = os.path.join(SRC_DIR, "impl")

# All the .cpp sources: core RVO2 + RL extensions + your binding.cpp
sources = [
    os.path.join(IMPL_DIR, "Agent.cpp"),
    os.path.join(IMPL_DIR, "KdTree.cpp"),
    os.path.join(IMPL_DIR, "Obstacle.cpp"),
    os.path.join(IMPL_DIR, "RVOSimulator_core.cpp"),
    os.path.join(IMPL_DIR, "RVOSimulator_rl_extensions.cpp"),
    os.path.join(IMPL_DIR, "RayCastingEngine.cpp"),
    os.path.join(INCLUDE_DIR, "binding.cpp"),
]

compile_args = [
    "-O3",              # 🚀 máxima optimización
    "-march=native",    # usa las instrucciones específicas de tu CPU
    "-ffast-math",      # operaciones de punto flotante más agresivas
    "-fopenmp",         # soporte multihilo
    "-std=c++17",
    "-fopt-info-vec=vec_report.txt"
]

ext_modules = [
    Extension(
        name="rvo2_rl",           # must match the PYBIND11_MODULE name in binding.cpp
        sources=sources,
        include_dirs=[
            INCLUDE_DIR,        # your C++ headers
            pybind11.get_include(),  # pybind11 headers
        ],
        language="c++",
        extra_compile_args=compile_args,
        extra_link_args=["-fopenmp"],
    ),
]

setup(
    name="rvo2_rl",
    version="0.1.0",
    author="Felipe Cerda",
    description="Python bindings for RVO2 extended for reinforcement‑learning",
    python_requires=">=3.11",
    ext_modules=ext_modules,
    zip_safe=False,
)
