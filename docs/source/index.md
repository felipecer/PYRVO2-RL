# PYRVO2-RL Documentation

**Python bindings for RVO2 extended for reinforcement learning research**

> 🔥 **Live Documentation Server Active!** Changes to this documentation are automatically rebuilt and refreshed in your browser.

PYRVO2-RL is a high-performance multi-agent collision avoidance simulator specifically designed for reinforcement learning experiments. It extends the popular RVO2 (Reciprocal Velocity Obstacles) algorithm with RL-friendly observation and control interfaces, making it ideal for training and evaluating multi-agent RL algorithms.

## Key Features

- **RL-Optimized**: Custom observation modes (Cartesian, Polar) and batch processing
- **High Performance**: OpenMP parallelization and optimized C++ implementation  
- **Flexible Sensing**: Configurable neighbor detection and LiDAR-style observations
- **Research Ready**: Direct integration with popular RL frameworks (Gymnasium, etc.)
- **Cross-Platform**: Support for Linux and macOS with minimal dependencies

## Quick Start

```python
import rvo2_rl

# Create RL-enabled simulator
wrapper = rvo2_rl.rl.RVO2RLWrapper(
    time_step=0.25,
    neighbor_dist=15.0,
    max_neighbors=10,
    mode=rvo2_rl.rl.ObsMode.Cartesian
)

# Add agents and get observations
agent_id = wrapper.add_agent((0, 0))
observation = wrapper.get_observation(agent_id)
```

## Documentation Structure

```{toctree}
:maxdepth: 2
:caption: Contents:

installation
quickstart
api/index
examples/index
technical/index
```

```{toctree}
:maxdepth: 2
:caption: API Reference

api/rl_extensions
api/rvo2_core
api/utilities
```

```{toctree}
:maxdepth: 2
:caption: Usage Examples

examples/basic_usage
examples/multi_agent
examples/advanced
```

```{toctree}
:maxdepth: 2
:caption: Technical Details

technical/architecture
technical/performance
```

## Project Information

- **Version**: 0.1.0
- **Author**: Felipe Cerda
- **License**: MIT
- **Python Requirements**: >=3.10
- **Source Code**: [GitHub Repository](https://github.com/felipecer/PYRVO2-RL)

## Research Context

PYRVO2-RL bridges the gap between collision avoidance research and reinforcement learning by providing:

1. **Standardized Observation Interfaces**: Consistent observation formats across different sensing modalities
2. **Scalable Multi-Agent Support**: Efficient handling of dozens to hundreds of agents
3. **Customizable Dynamics**: Tunable parameters for different experimental scenarios
4. **Integration Examples**: Ready-to-use patterns for common RL frameworks

Whether you're researching crowd simulation, autonomous vehicle coordination, or general multi-agent reinforcement learning, PYRVO2-RL provides the performance and flexibility needed for serious research.

## Indices and Tables

- {ref}`genindex`
- {ref}`modindex`
- {ref}`search`
