import os
import sys
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
own_impl_sources = glob.glob(os.path.join(OWN_IMPL_DIR, "*.cpp"))
rvo2_sources = glob.glob(os.path.join(RVO2_IMPL_DIR, "*.cpp"))
main_binding = os.path.join(SRC_DIR, "main_bindings.cpp")

sources = [main_binding] + binding_sources + own_impl_sources + rvo2_sources

# Platform-specific OpenMP flags
compile_args = ["-O3", "-march=native", "-ffast-math", "-std=c++17"]
link_args = []

if sys.platform == "darwin":
    # macOS (use Homebrew's LLVM for OpenMP)
    llvm_dir = "/opt/homebrew/opt/llvm" if os.uname().machine == "arm64" else "/usr/local/opt/llvm"
    compile_args += [
        "-Xpreprocessor", "-fopenmp",
        f"-I{llvm_dir}/include"
    ]
    link_args += [
        f"-L{llvm_dir}/lib",
        "-lomp"
    ]
else:
    # Linux
    compile_args += ["-fopenmp", "-fopt-info-vec=vec_report.txt"]
    link_args += ["-fopenmp"]

ext_modules = [
    Extension(
        name="rvo2_rl",
        sources=sources,
        include_dirs=[
            ".", SRC_DIR, RVO2_INCLUDE_DIR, INCLUDE_DIR,
            pybind11.get_include(),
        ],
        language="c++",
        extra_compile_args=compile_args,
        extra_link_args=link_args,
    ),
]

setup(
    name="pyrvo2_rl",
    version="0.1.0",
    author="Felipe Cerda",
    description="Python bindings for RVO2 extended for reinforcement-learning",
    python_requires=">=3.10",
    ext_modules=ext_modules,
    zip_safe=False,
)
