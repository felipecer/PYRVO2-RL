# Installation Guide

This guide covers the installation of PYRVO2-RL on Linux and macOS systems.

## System Requirements

- **Python**: 3.10 or higher
- **C++ Compiler**: GCC 7+ or Clang 8+ with C++17 support
- **Build Tools**: CMake 3.15+ (automatically handled by pip)
- **OpenMP**: Platform-specific OpenMP implementation

## Platform-Specific Dependencies

### Linux (Ubuntu/Debian)

```bash
# Install build essentials and OpenMP
sudo apt-get update
sudo apt-get install build-essential libomp-dev

# For Python development headers (if needed)
sudo apt-get install python3-dev
```

### Linux (CentOS/RHEL/Fedora)

```bash
# CentOS/RHEL
sudo yum install gcc-c++ libomp-devel

# Fedora
sudo dnf install gcc-c++ libomp-devel
```

### macOS

```bash
# Install Homebrew if not already installed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install LLVM for OpenMP support
brew install llvm

# Add LLVM to your shell profile
echo 'export PATH="/opt/homebrew/opt/llvm/bin:$PATH"' >> ~/.zshrc
echo 'export LDFLAGS="-L/opt/homebrew/opt/llvm/lib"' >> ~/.zshrc
echo 'export CPPFLAGS="-I/opt/homebrew/opt/llvm/include"' >> ~/.zshrc

# Reload your shell
source ~/.zshrc
```

## Installation Methods

### Method 1: Install from Source (Recommended for Development)

```bash
# Clone the repository
git clone https://github.com/felipecer/PYRVO2-RL.git
cd PYRVO2-RL

# Create a virtual environment (recommended)
python -m venv venv
source venv/bin/activate  # On Windows: venv\\Scripts\\activate

# Install in development mode
pip install -e .
```

### Method 2: Direct Installation

```bash
# Install directly from the repository
pip install git+https://github.com/felipecer/PYRVO2-RL.git
```

### Method 3: Local Wheel Build

```bash
# Build a wheel package
python -m pip install build
python -m build

# Install the built wheel
pip install dist/pyrvo2_rl-*.whl
```

## Verification

Test your installation with this simple script:

```python
import rvo2_rl

# Test core functionality
print("✓ Core module imported successfully")

# Test RL extensions
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
print("✓ RL extensions imported successfully")

# Test basic functionality
wrapper = RVO2RLWrapper()
print("✓ RVO2RLWrapper created successfully")

# Test Vector2 operations
from rvo2_rl.rvo2 import Vector2
v = Vector2(1.0, 2.0)
print(f"✓ Vector2 operations work: {v}")

print("Installation verification complete!")
```

## Build Configuration

PYRVO2-RL uses optimized build settings by default:

- **Optimization Level**: -O3 for maximum performance
- **Architecture**: -march=native for CPU-specific optimizations
- **Math Optimizations**: -ffast-math for floating-point speed
- **Parallelization**: OpenMP for multi-threading
- **C++ Standard**: C++17

## Troubleshooting

### Common Build Issues

#### 1. OpenMP Not Found

**Symptoms**: Build fails with OpenMP-related errors

**Linux Solution**:
```bash
sudo apt-get install libomp-dev
```

**macOS Solution**:
```bash
brew install llvm
export PATH="/opt/homebrew/opt/llvm/bin:$PATH"
```

#### 2. C++17 Support Missing

**Symptoms**: Compiler errors about C++17 features

**Solution**: Upgrade your compiler:
```bash
# Ubuntu/Debian
sudo apt-get install gcc-9 g++-9

# Set as default
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 90
```

#### 3. Python Header Files Missing

**Symptoms**: Build fails with Python.h not found

**Solution**:
```bash
# Ubuntu/Debian
sudo apt-get install python3-dev

# CentOS/RHEL
sudo yum install python3-devel
```

#### 4. Memory Issues During Build

**Symptoms**: Build fails with "virtual memory exhausted"

**Solution**: Reduce optimization level temporarily:
```bash
export CXXFLAGS="-O2"  # Instead of -O3
pip install .
```

### Performance Verification

Test the performance of your installation:

```python
import time
import numpy as np
from rvo2_rl.rl import RVO2RLWrapper
from rvo2_rl.rvo2 import Vector2

def benchmark_simulation():
    # Create wrapper with moderate settings
    wrapper = RVO2RLWrapper(
        neighbor_dist=10.0,
        max_neighbors=5,
        time_step=0.1
    )
    
    # Add 100 agents
    for i in range(100):
        x = np.random.uniform(-10, 10)
        y = np.random.uniform(-10, 10)
        wrapper.add_agent(Vector2(x, y))
    
    # Set random goals
    goals = [Vector2(np.random.uniform(-10, 10), 
                     np.random.uniform(-10, 10)) 
             for _ in range(100)]
    wrapper.set_goals(goals)
    wrapper.initialize()
    
    # Benchmark 1000 simulation steps
    start_time = time.time()
    for _ in range(1000):
        wrapper.set_preferred_velocities()
        wrapper.get_simulator().do_step()
    
    elapsed = time.time() - start_time
    steps_per_second = 1000 / elapsed
    
    print(f"Performance: {steps_per_second:.1f} steps/second with 100 agents")
    
    if steps_per_second > 500:
        print("✓ Excellent performance")
    elif steps_per_second > 100:
        print("✓ Good performance")
    else:
        print("⚠ Performance may be suboptimal - check OpenMP installation")

if __name__ == "__main__":
    benchmark_simulation()
```

### Environment Variables

You can customize the build process with these environment variables:

```bash
# Disable OpenMP (not recommended)
export PYRVO2_DISABLE_OPENMP=1

# Custom compiler flags
export CXXFLAGS="-O2 -march=native"

# Force specific compiler
export CXX=g++-9

# Install with custom settings
pip install .
```

## Next Steps

Once installation is complete, proceed to the [Quick Start Guide](quickstart.md) for your first PYRVO2-RL experiment.

For integration with specific RL frameworks, see our [Usage Examples](examples/basic_usage.md).
