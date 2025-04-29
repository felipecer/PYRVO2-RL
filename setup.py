import os
import glob
from setuptools import setup, Extension
import pybind11

# Base directories
SRC_DIR = "src"
BINDINGS_DIR = os.path.join(SRC_DIR, "bindings")
OWN_IMPL_DIR = os.path.join(SRC_DIR, "impl")
RVO2_IMPL_DIR = os.path.join(SRC_DIR, "rvo2", "impl")
RVO2_INCLUDE_DIR = os.path.join(SRC_DIR, "rvo2", "include")
INCLUDE_DIR = os.path.join(SRC_DIR, "include")

# Gather all the .cpp files
binding_sources = glob.glob(os.path.join(BINDINGS_DIR, "*.cpp"))
own_impl_sources = glob.glob(os.path.join(OWN_IMPL_DIR,    "*.cpp"))
rvo2_sources = glob.glob(os.path.join(RVO2_IMPL_DIR,   "*.cpp"))
main_binding = os.path.join(SRC_DIR, "main_bindings.cpp")

sources = [main_binding] + binding_sources + own_impl_sources + rvo2_sources

# Compiler flags
compile_args = [
    "-O3",
    "-march=native",
    "-ffast-math",
    "-fopenmp",
    "-std=c++17",
    "-fopt-info-vec=vec_report.txt",
]

ext_modules = [
    Extension(
        # debe coincidir con PYBIND11_MODULE(rvo2_rl, ...)
        name="rvo2_rl",
        sources=sources,
        include_dirs=[
            ".",                   # para includes como "src/rvo2/include/..."
            SRC_DIR,               # por si incluyes directamente "rvo2/..."
            INCLUDE_DIR,           # tus headers propios en src/include/
            RVO2_INCLUDE_DIR,      # headers vendorizados de RVO2
            pybind11.get_include(),  # pybind11 headers
        ],
        language="c++",
        extra_compile_args=compile_args,
        extra_link_args=["-fopenmp"],
    ),
]

setup(
    name="pyrvo2_rl",
    version="0.1.0",
    author="Felipe Cerda",
    description="Python bindings for RVO2 extended for reinforcement-learning",
    python_requires=">=3.11",
    ext_modules=ext_modules,
    zip_safe=False,
)
