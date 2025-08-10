# PYRVO2-RL Documentation Testing Project

## Project Overview

This document describes a **step-by-step** testing project designed to validate PYRVO2-RL documentation incrementally. Each phase is self-contained and manageable, preventing context overflow while ensuring thorough validation.

**Testing Project Location**: `doc_testing_project/` (excluded from version control)

## Core Principles

### 🎯 **Step-by-Step Approach**
- **One phase at a time**: Complete each phase before moving to the next
- **Isolated environments**: Each test uses fresh conda environments
- **Incremental validation**: Build confidence progressively
- **Context-friendly**: Each step fits within reasonable limits

### 🔧 **Environment Strategy**
- **Conda-based**: Use conda environments for reproducibility
- **Clean installs**: Each test starts with a fresh environment
- **Pip installation**: Install PYRVO2-RL via pip in each environment
- **Dependency isolation**: Prevent conflicts between tests

### 📊 **Validation Targets**
1. **Phase 1**: Basic installation and import testing
2. **Phase 2**: Core API functionality testing  
3. **Phase 3**: Simple example validation
4. **Phase 4**: Multi-agent scenario testing
5. **Phase 5**: Performance validation
6. **Phase 6**: Integration testing

## Testing Project Structure

```
doc_testing_project/
├── README.md                           # Testing overview and instructions
├── requirements.txt                    # Base testing dependencies
├── phase_runner.py                     # Single phase test runner
├── test_config.yaml                    # Configuration for tests
├── phase_1_installation/
│   ├── test_basic_install.py          # Basic installation test
│   ├── test_import_validation.py      # Import and basic functionality
│   └── conda_env_setup.sh             # Environment setup script
├── phase_2_core_api/
│   ├── test_wrapper_creation.py       # RVO2RLWrapper basic tests
│   ├── test_agent_management.py       # Add agents, set goals
│   ├── test_observations.py           # Basic observation testing
│   └── conda_env_setup.sh             # Fresh environment for API tests
├── phase_3_simple_examples/
│   ├── test_quickstart_basic.py       # First simulation from quickstart
│   ├── test_two_agent_swap.py         # Simple two-agent example
│   └── conda_env_setup.sh             # Fresh environment for examples
├── phase_4_multi_agent/
│   ├── test_formation_basic.py        # Simple formation control
│   ├── test_multiple_agents.py        # 4+ agent scenarios
│   └── conda_env_setup.sh             # Environment for multi-agent tests
├── phase_5_performance/
│   ├── test_scaling_basic.py          # Basic scaling test (10-100 agents)
│   ├── test_benchmark_simple.py       # Simple performance benchmark
│   └── conda_env_setup.sh             # Environment for performance tests
└── phase_6_integration/
    ├── test_gymnasium_basic.py        # Basic Gym environment test
    ├── test_simple_training.py        # Minimal RL training test
    └── conda_env_setup.sh             # Environment with RL dependencies
```

## Phase-by-Phase Implementation

### Phase 1: Basic Installation and Import Testing

**Objective**: Verify that PYRVO2-RL can be installed and imported successfully

#### 1.1 Environment Setup Script
```bash
# phase_1_installation/conda_env_setup.sh
#!/bin/bash

set -e  # Exit on any error

echo "Setting up Phase 1 testing environment..."

# Remove existing environment if it exists
conda env remove -n pyrvo2rl_test_phase1 -y 2>/dev/null || true

# Create fresh conda environment
conda create -n pyrvo2rl_test_phase1 python=3.11 -y

# Activate environment
eval "$(conda shell.bash hook)"
conda activate pyrvo2rl_test_phase1

# Install build dependencies
conda install -c conda-forge build-essential cmake -y
pip install --upgrade pip setuptools wheel

# Install PYRVO2-RL from source (assuming we're in the project root)
cd ../..  # Go back to project root
pip install -e .

echo "Phase 1 environment setup complete!"
echo "Environment name: pyrvo2rl_test_phase1"
```

#### 1.2 Basic Installation Test
```python
# phase_1_installation/test_basic_install.py
import subprocess
import sys
import os

def test_conda_environment():
    """Test that conda environment was created successfully."""
    result = subprocess.run(['conda', 'env', 'list'], 
                          capture_output=True, text=True)
    assert 'pyrvo2rl_test_phase1' in result.stdout, "Conda environment not found"
    print("✓ Conda environment exists")

def test_python_version():
    """Test Python version in environment."""
    assert sys.version_info >= (3, 10), f"Python version too old: {sys.version}"
    print(f"✓ Python version: {sys.version}")

def test_basic_import():
    """Test basic import of rvo2_rl."""
    try:
        import rvo2_rl
        print("✓ Successfully imported rvo2_rl")
        return True
    except ImportError as e:
        print(f"✗ Failed to import rvo2_rl: {e}")
        return False

def test_rl_import():
    """Test import of RL extensions."""
    try:
        from rvo2_rl.rl import RVO2RLWrapper, ObsMode
        print("✓ Successfully imported RL extensions")
        return True
    except ImportError as e:
        print(f"✗ Failed to import RL extensions: {e}")
        return False

def test_core_import():
    """Test import of core RVO2."""
    try:
        from rvo2_rl.rvo2 import Vector2
        print("✓ Successfully imported RVO2 core")
        return True
    except ImportError as e:
        print(f"✗ Failed to import RVO2 core: {e}")
        return False

def run_phase_1_tests():
    """Run all Phase 1 tests."""
    print("="*50)
    print("PHASE 1: BASIC INSTALLATION TESTING")
    print("="*50)
    
    tests = [
        test_conda_environment,
        test_python_version,
        test_basic_import,
        test_rl_import,
        test_core_import
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            result = test()
            if result is not False:
                passed += 1
        except Exception as e:
            print(f"✗ {test.__name__} failed: {e}")
            failed += 1
    
    print("\n" + "="*50)
    print(f"PHASE 1 RESULTS: {passed} passed, {failed} failed")
    print("="*50)
    
    return failed == 0

if __name__ == "__main__":
    success = run_phase_1_tests()
    sys.exit(0 if success else 1)
```

### Phase 2: Core API Functionality Testing

**Objective**: Test basic RVO2RLWrapper functionality

#### 2.1 Environment Setup
```bash
# phase_2_core_api/conda_env_setup.sh
#!/bin/bash

set -e

echo "Setting up Phase 2 testing environment..."

# Remove existing environment
conda env remove -n pyrvo2rl_test_phase2 -y 2>/dev/null || true

# Create fresh environment
conda create -n pyrvo2rl_test_phase2 python=3.11 -y

# Activate environment
eval "$(conda shell.bash hook)"
conda activate pyrvo2rl_test_phase2

# Install dependencies
conda install -c conda-forge build-essential cmake numpy -y
pip install --upgrade pip setuptools wheel

# Install PYRVO2-RL
cd ../..
pip install -e .

echo "Phase 2 environment setup complete!"
```

#### 2.2 Wrapper Creation Test
```python
# phase_2_core_api/test_wrapper_creation.py
import sys
import numpy as np

def test_import_requirements():
    """Test all required imports."""
    try:
        from rvo2_rl.rl import RVO2RLWrapper, ObsMode
        from rvo2_rl.rvo2 import Vector2
        print("✓ All imports successful")
        return True
    except ImportError as e:
        print(f"✗ Import failed: {e}")
        return False

def test_default_wrapper_creation():
    """Test creating wrapper with default parameters."""
    try:
        from rvo2_rl.rl import RVO2RLWrapper
        wrapper = RVO2RLWrapper()
        print("✓ Default wrapper created successfully")
        return True
    except Exception as e:
        print(f"✗ Default wrapper creation failed: {e}")
        return False

def test_parameterized_wrapper_creation():
    """Test creating wrapper with specific parameters."""
    try:
        from rvo2_rl.rl import RVO2RLWrapper, ObsMode
        wrapper = RVO2RLWrapper(
            time_step=0.25,
            neighbor_dist=10.0,
            max_neighbors=5,
            radius=0.5,
            max_speed=2.0,
            mode=ObsMode.Cartesian
        )
        print("✓ Parameterized wrapper created successfully")
        return True
    except Exception as e:
        print(f"✗ Parameterized wrapper creation failed: {e}")
        return False

def test_agent_addition():
    """Test adding agents to the simulation."""
    try:
        from rvo2_rl.rl import RVO2RLWrapper
        from rvo2_rl.rvo2 import Vector2
        
        wrapper = RVO2RLWrapper()
        
        # Add a single agent
        agent_id = wrapper.add_agent(Vector2(0.0, 0.0))
        assert agent_id == 0, f"Expected agent ID 0, got {agent_id}"
        
        # Add second agent
        agent_id_2 = wrapper.add_agent(Vector2(1.0, 1.0))
        assert agent_id_2 == 1, f"Expected agent ID 1, got {agent_id_2}"
        
        print("✓ Agent addition successful")
        return True
    except Exception as e:
        print(f"✗ Agent addition failed: {e}")
        return False

def test_goal_setting():
    """Test setting goals for agents."""
    try:
        from rvo2_rl.rl import RVO2RLWrapper
        from rvo2_rl.rvo2 import Vector2
        
        wrapper = RVO2RLWrapper()
        wrapper.add_agent(Vector2(0.0, 0.0))
        wrapper.add_agent(Vector2(1.0, 1.0))
        
        # Set goals
        goals = [Vector2(2.0, 2.0), Vector2(-1.0, -1.0)]
        wrapper.set_goals(goals)
        
        print("✓ Goal setting successful")
        return True
    except Exception as e:
        print(f"✗ Goal setting failed: {e}")
        return False

def test_initialization():
    """Test wrapper initialization."""
    try:
        from rvo2_rl.rl import RVO2RLWrapper
        from rvo2_rl.rvo2 import Vector2
        
        wrapper = RVO2RLWrapper()
        wrapper.add_agent(Vector2(0.0, 0.0))
        wrapper.set_goals([Vector2(1.0, 1.0)])
        
        # Initialize should not raise an exception
        wrapper.initialize()
        
        print("✓ Wrapper initialization successful")
        return True
    except Exception as e:
        print(f"✗ Wrapper initialization failed: {e}")
        return False

def run_phase_2_tests():
    """Run all Phase 2 tests."""
    print("="*50)
    print("PHASE 2: CORE API FUNCTIONALITY TESTING")
    print("="*50)
    
    tests = [
        test_import_requirements,
        test_default_wrapper_creation,
        test_parameterized_wrapper_creation,
        test_agent_addition,
        test_goal_setting,
        test_initialization
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            result = test()
            if result is not False:
                passed += 1
        except Exception as e:
            print(f"✗ {test.__name__} failed: {e}")
            failed += 1
    
    print("\n" + "="*50)
    print(f"PHASE 2 RESULTS: {passed} passed, {failed} failed")
    print("="*50)
    
    return failed == 0

if __name__ == "__main__":
    success = run_phase_2_tests()
    sys.exit(0 if success else 1)
```

### Phase 3: Simple Example Testing

**Objective**: Test the simplest examples from documentation

#### 3.1 Environment Setup
```bash
# phase_3_simple_examples/conda_env_setup.sh
#!/bin/bash

set -e

echo "Setting up Phase 3 testing environment..."

conda env remove -n pyrvo2rl_test_phase3 -y 2>/dev/null || true
conda create -n pyrvo2rl_test_phase3 python=3.11 -y

eval "$(conda shell.bash hook)"
conda activate pyrvo2rl_test_phase3

conda install -c conda-forge build-essential cmake numpy -y
pip install --upgrade pip setuptools wheel

cd ../..
pip install -e .

echo "Phase 3 environment setup complete!"
```

#### 3.2 Two-Agent Swap Test
```python
# phase_3_simple_examples/test_two_agent_swap.py
import sys

def test_two_agent_swap():
    """Test the two-agent swap example from documentation."""
    try:
        from rvo2_rl.rl import RVO2RLWrapper, ObsMode
        from rvo2_rl.rvo2 import Vector2
        
        # Create wrapper exactly as in documentation
        wrapper = RVO2RLWrapper(
            time_step=0.25,
            neighbor_dist=10.0,
            max_neighbors=5,
            radius=0.5,
            max_speed=1.5,
            mode=ObsMode.Cartesian
        )
        
        # Add two agents at opposite positions
        agent1_id = wrapper.add_agent(Vector2(-5.0, 0.0))
        agent2_id = wrapper.add_agent(Vector2(5.0, 0.0))
        
        # Set goals (they want to swap positions)
        wrapper.set_goals([Vector2(5.0, 0.0), Vector2(-5.0, 0.0)])
        wrapper.initialize()
        
        print("✓ Two-agent scenario set up successfully")
        
        # Run simulation for a few steps
        for step in range(10):
            # Get observations (should not crash)
            obs1 = wrapper.get_observation(agent1_id)
            obs2 = wrapper.get_observation(agent2_id)
            
            assert obs1 is not None, "Agent 1 observation is None"
            assert obs2 is not None, "Agent 2 observation is None"
            assert len(obs1.shape) == 1, "Observation should be 1D array"
            assert len(obs2.shape) == 1, "Observation should be 1D array"
            
            # Use automatic goal-seeking behavior
            wrapper.set_preferred_velocities()
            wrapper.get_simulator().do_step()
            
            # Get distances to goals
            dist1 = wrapper.get_distance_to_goal(agent1_id)
            dist2 = wrapper.get_distance_to_goal(agent2_id)
            
            assert dist1 >= 0, "Distance to goal should be non-negative"
            assert dist2 >= 0, "Distance to goal should be non-negative"
        
        print("✓ Two-agent simulation completed successfully")
        return True
        
    except Exception as e:
        print(f"✗ Two-agent swap test failed: {e}")
        return False

def test_observation_structure():
    """Test basic observation structure."""
    try:
        from rvo2_rl.rl import RVO2RLWrapper
        from rvo2_rl.rvo2 import Vector2
        
        wrapper = RVO2RLWrapper()
        wrapper.add_agent(Vector2(0.0, 0.0))
        wrapper.set_goals([Vector2(1.0, 1.0)])
        wrapper.initialize()
        
        # Get observation bounds
        bounds = wrapper.get_observation_bounds()
        
        assert 'low' in bounds, "Bounds should contain 'low' key"
        assert 'high' in bounds, "Bounds should contain 'high' key"
        assert 'info' in bounds, "Bounds should contain 'info' key"
        
        # Get actual observation
        obs = wrapper.get_observation(0)
        
        assert len(obs) == len(bounds['low']), "Observation length should match bounds"
        assert len(obs) == len(bounds['high']), "Observation length should match bounds"
        
        print("✓ Observation structure validation successful")
        return True
        
    except Exception as e:
        print(f"✗ Observation structure test failed: {e}")
        return False

def run_phase_3_tests():
    """Run all Phase 3 tests."""
    print("="*50)
    print("PHASE 3: SIMPLE EXAMPLE TESTING")
    print("="*50)
    
    tests = [
        test_two_agent_swap,
        test_observation_structure
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            result = test()
            if result is not False:
                passed += 1
        except Exception as e:
            print(f"✗ {test.__name__} failed: {e}")
            failed += 1
    
    print("\n" + "="*50)
    print(f"PHASE 3 RESULTS: {passed} passed, {failed} failed")
    print("="*50)
    
    return failed == 0

if __name__ == "__main__":
    success = run_phase_3_tests()
    sys.exit(0 if success else 1)
```

### Master Phase Runner

```python
# phase_runner.py
#!/usr/bin/env python3
"""
Single-phase test runner for PYRVO2-RL documentation validation.
Usage: python phase_runner.py <phase_number>
"""

import sys
import subprocess
import os
from pathlib import Path

class PhaseRunner:
    """Run a single phase of documentation testing."""
    
    def __init__(self):
        self.phases = {
            1: {
                'name': 'Basic Installation',
                'dir': 'phase_1_installation',
                'setup_script': 'conda_env_setup.sh',
                'test_script': 'test_basic_install.py'
            },
            2: {
                'name': 'Core API',
                'dir': 'phase_2_core_api', 
                'setup_script': 'conda_env_setup.sh',
                'test_script': 'test_wrapper_creation.py'
            },
            3: {
                'name': 'Simple Examples',
                'dir': 'phase_3_simple_examples',
                'setup_script': 'conda_env_setup.sh', 
                'test_script': 'test_two_agent_swap.py'
            }
        }
    
    def run_phase(self, phase_num):
        """Run a specific phase."""
        if phase_num not in self.phases:
            print(f"Error: Phase {phase_num} not found!")
            print(f"Available phases: {list(self.phases.keys())}")
            return False
        
        phase = self.phases[phase_num]
        phase_dir = Path(phase['dir'])
        
        print(f"Running Phase {phase_num}: {phase['name']}")
        print("="*60)
        
        # Step 1: Environment setup
        print("Step 1: Setting up conda environment...")
        setup_script = phase_dir / phase['setup_script']
        
        if setup_script.exists():
            result = subprocess.run(['bash', str(setup_script)], 
                                  capture_output=True, text=True)
            if result.returncode != 0:
                print(f"Environment setup failed: {result.stderr}")
                return False
            print("✓ Environment setup completed")
        else:
            print(f"Warning: Setup script {setup_script} not found")
        
        # Step 2: Run tests
        print("Step 2: Running tests...")
        test_script = phase_dir / phase['test_script']
        
        if test_script.exists():
            # Activate the environment and run tests
            env_name = f"pyrvo2rl_test_phase{phase_num}"
            cmd = [
                'conda', 'run', '-n', env_name, 'python', str(test_script)
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            print(result.stdout)
            if result.stderr:
                print("STDERR:", result.stderr)
            
            if result.returncode == 0:
                print(f"✓ Phase {phase_num} completed successfully!")
                return True
            else:
                print(f"✗ Phase {phase_num} failed!")
                return False
        else:
            print(f"Error: Test script {test_script} not found")
            return False
    
    def list_phases(self):
        """List available phases."""
        print("Available test phases:")
        for num, phase in self.phases.items():
            print(f"  {num}: {phase['name']}")

def main():
    runner = PhaseRunner()
    
    if len(sys.argv) != 2:
        print("Usage: python phase_runner.py <phase_number>")
        print()
        runner.list_phases()
        sys.exit(1)
    
    try:
        phase_num = int(sys.argv[1])
    except ValueError:
        print("Error: Phase number must be an integer")
        runner.list_phases()
        sys.exit(1)
    
    success = runner.run_phase(phase_num)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
```

## Quick Start Guide

### Running Individual Phases

```bash
# Create the testing project directory
mkdir -p doc_testing_project
cd doc_testing_project

# Copy the phase runner
# (copy phase_runner.py and create phase directories)

# Run Phase 1: Basic Installation
python phase_runner.py 1

# If Phase 1 passes, run Phase 2
python phase_runner.py 2

# Continue with subsequent phases
python phase_runner.py 3
```

### Phase Success Criteria

**Phase 1 Success**: All imports work, conda environment is created
**Phase 2 Success**: Wrapper creation, agent addition, goal setting work
**Phase 3 Success**: Simple two-agent example runs without errors

## Benefits of This Approach

### 🎯 **Manageable Scope**
- Each phase is small and focused
- Easy to debug individual issues
- Context window stays reasonable

### 🔧 **Clean Environments**  
- Fresh conda environment for each phase
- Proper pip installation of PYRVO2-RL
- Isolated dependencies prevent conflicts

### 📈 **Progressive Validation**
- Build confidence step by step
- Early detection of fundamental issues
- Clear success/failure criteria

### 🚀 **Easy Execution**
- Simple command: `python phase_runner.py <phase>`
- Automated environment setup
- Clear success/failure reporting

## Next Steps: Expanding the Testing Project

Once the first 3 phases are working, we can add:

### Phase 4: Multi-Agent Scenarios (4+ agents)
- Formation control tests
- Simple swarm behavior validation  
- Multi-agent observation testing

### Phase 5: Performance Validation
- Basic scaling tests (10, 50, 100 agents)
- Simple benchmark validation
- Memory usage monitoring

### Phase 6: Integration Testing  
- Basic Gymnasium environment wrapper
- Simple RL training loop test
- Framework compatibility validation

## Implementation Notes

### 🔧 **Environment Management**
- Each phase uses a **fresh conda environment**
- Proper **pip installation** of PYRVO2-RL in each environment
- **Clean slate** approach prevents contamination between phases

### 📊 **Validation Approach**
- **Incremental complexity**: Start simple, add complexity gradually
- **Clear success criteria**: Each test has obvious pass/fail conditions
- **Context-aware**: Each phase fits within manageable limits

### 🎯 **Focus Areas**
- **Phase 1-3**: Core functionality that must work
- **Phase 4-6**: Advanced features and integrations
- **Modular design**: Can stop at any phase if issues are found

This approach ensures we can methodically validate the documentation without overwhelming the context window, while using proper conda environments and pip installation as required.
