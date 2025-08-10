# API Reference

This section provides comprehensive API documentation for all PYRVO2-RL components.

## Overview

PYRVO2-RL provides three main API layers:

1. **RL Extensions** - High-level interface optimized for reinforcement learning
2. **RVO2 Core** - Low-level collision avoidance and simulation control  
3. **Utilities** - Performance monitoring and optimization tools

## Quick Navigation

- **For RL Research:** Start with [RL Extensions](rl_extensions.md) for Gymnasium integration and observation processing
- **For Custom Environments:** Use [RVO2 Core](rvo2_core.md) for fine-grained simulation control
- **For Performance Tuning:** Check [Utilities](utilities.md) for profiling and optimization

```{toctree}
:maxdepth: 2

rl_extensions
rvo2_core
utilities
```

## Common Usage Patterns

### Basic RL Environment Setup
```python
from rvo2_rl.rl import RVO2RLWrapper, ObsMode

# Create wrapper for RL applications
wrapper = RVO2RLWrapper(
    mode=ObsMode.Cartesian,
    use_lidar=True,
    use_obs_mask=True
)

# Add agents and set goals
agent_id = wrapper.add_agent(Vector2(0, 0))
wrapper.set_goals([Vector2(10, 10)])
wrapper.initialize()

# Get observations for RL training
observations = wrapper.get_observations()
```

### Low-Level Simulation Control
```python
from rvo2_rl.rvo2 import RVOSimulator, Vector2

# Direct simulator access
sim = RVOSimulator()
agent_id = sim.addAgent(Vector2(0, 0))
sim.setAgentPrefVelocity(agent_id, Vector2(1, 0))
sim.doStep()
```

### Performance Monitoring
```python
from rvo2_rl.util import profile_simulation, openmp_threads_used

# Monitor simulation performance
stats = profile_simulation(wrapper, num_steps=100)
print(f"Performance: {stats['steps_per_second']:.1f} FPS")
print(f"Threading: {openmp_threads_used()} threads")
```

## Data Structures

### Key Classes

| Class | Purpose | Documentation |
|-------|---------|---------------|
| `RVO2RLWrapper` | Main RL interface | [RL Extensions](rl_extensions.md) |
| `RVOSimulator` | Core simulation engine | [RVO2 Core](rvo2_core.md) |
| `Vector2` | 2D vector operations | [RVO2 Core](rvo2_core.md) |
| `ObsMode` | Observation format enum | [RL Extensions](rl_extensions.md) |

### Observation Formats

The library supports multiple observation formats optimized for different RL algorithms:

- **Cartesian:** `(rel_x, rel_y, vel_x, vel_y, [mask])` - Best for CNN/MLP
- **Polar:** `(distance, angle, vel_radial, vel_tangential, [mask])` - Rotation invariant
- **Raw:** `(abs_x, abs_y, vel_x, vel_y, [mask])` - Global coordinates

Each format includes optional validity masks for handling variable numbers of neighbors.

## Error Handling

Common error patterns and solutions:

```python
try:
    wrapper = RVO2RLWrapper(...)
    wrapper.initialize()
except ValueError as e:
    print(f"Configuration error: {e}")
except RuntimeError as e:
    print(f"Simulation error: {e}")
```

## Thread Safety

PYRVO2-RL is designed for single-threaded operation from Python, but uses OpenMP internally for parallel computation. Do not access the same simulator instance from multiple Python threads simultaneously.

For parallel RL training, create separate simulator instances for each environment:

```python
# Safe parallel usage
envs = [RVO2RLWrapper(...) for _ in range(num_parallel_envs)]
```
