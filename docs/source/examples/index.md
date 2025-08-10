# Examples & Tutorials

This section provides practical examples and tutorials for using PYRVO2-RL in various scenarios, from basic multi-agent coordination to advanced research applications.

## Learning Path

We recommend following this progression:

1. **Basic Usage** - Start here to understand fundamental concepts
2. **Multi-Agent Scenarios** - Learn coordination and formation control
3. **Advanced Examples** - Explore LiDAR, optimization, and complex environments

```{toctree}
:maxdepth: 2

basic_usage
multi_agent
advanced
```

## Example Categories

### Basic Usage Examples
- **Two-Agent Swap:** Simple collision avoidance between two agents
- **Circle Formation:** Agents forming and maintaining circular formations
- **RL Action Integration:** Converting reinforcement learning actions to agent behaviors
- **Observation Analysis:** Understanding different observation formats
- **Custom Environment Creation:** Building Gymnasium-compatible environments

### Multi-Agent Scenarios
- **Formation Control:** Complex multi-agent formations (line, diamond, grid)
- **Swarm Behavior:** Leader-follower dynamics and emergent patterns
- **Large-Scale Evacuation:** Simulating crowd evacuation with 200+ agents
- **Competitive Resource Collection:** Multi-team competitive scenarios

### Advanced Features
- **LiDAR-Based Navigation:** Sensor-driven autonomous navigation
- **Dynamic Obstacle Avoidance:** Handling moving obstacles and environmental changes
- **Hierarchical Path Planning:** Combining global planning with local collision avoidance
- **Performance Optimization:** Scaling to 1000+ agents with optimal performance

## Code Organization

All examples follow a consistent pattern:

```python
# 1. Import required modules
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2

# 2. Create and configure wrapper
wrapper = RVO2RLWrapper(...)

# 3. Set up scenario (agents, goals, obstacles)
setup_scenario(wrapper)

# 4. Run simulation loop
for step in range(num_steps):
    # Update agent behaviors
    # Step simulation
    # Collect results
    
# 5. Analyze results
print_performance_summary()
```

## Running Examples

Each example is self-contained and can be run directly:

```bash
# Copy example code to a Python file
python basic_two_agent_example.py

# Or run in interactive environment
ipython
%run basic_two_agent_example.py
```

## Customization Guidelines

### Modifying Parameters
Most examples expose key parameters at the top for easy modification:

```python
# Configuration section
NUM_AGENTS = 50
SIMULATION_STEPS = 500
NEIGHBOR_DISTANCE = 6.0
MAX_SPEED = 1.5
```

### Adding Visualization
Examples focus on simulation logic. For visualization, add:

```python
import matplotlib.pyplot as plt

# Collect position data
positions = []
for step in range(num_steps):
    # ... simulation code ...
    positions.append(get_agent_positions())

# Plot trajectories
plot_agent_trajectories(positions)
```

### Performance Monitoring
All examples include basic performance reporting:

```python
start_time = time.time()
# ... simulation ...
elapsed = time.time() - start_time

print(f"Simulation completed in {elapsed:.2f}s")
print(f"Average FPS: {num_steps/elapsed:.1f}")
```

## Integration Examples

### Gymnasium Environment
```python
import gymnasium as gym
from your_examples import RVO2Environment

env = gym.make('RVO2-v0')
observation = env.reset()

for step in range(1000):
    action = env.action_space.sample()  # Random policy
    observation, reward, done, info = env.step(action)
    
    if done:
        observation = env.reset()
```

### Custom RL Training
```python
from stable_baselines3 import PPO
from your_examples import MultiAgentRVO2Env

env = MultiAgentRVO2Env(num_agents=10)
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100000)
```

## Research Applications

These examples demonstrate real research scenarios:

- **Crowd Dynamics:** Study pedestrian flow patterns and bottleneck formation
- **Swarm Robotics:** Test coordination algorithms for drone swarms
- **Game AI:** Develop intelligent NPC behavior for games
- **Urban Planning:** Simulate foot traffic in architectural designs
- **Emergency Response:** Model evacuation procedures and safety protocols

## Contributing Examples

We welcome community contributions! When adding examples:

1. Follow the established code structure
2. Include comprehensive comments
3. Add performance metrics
4. Test with different parameter ranges
5. Document any special requirements

See our [GitHub repository](https://github.com/felipecer/PYRVO2-RL) for contribution guidelines.

## Next Steps

After working through these examples:

- Explore the [API Reference](../api/index.md) for detailed function documentation
- Read [Technical Documentation](../technical/index.md) for implementation details
- Check [Performance Optimization](../technical/performance.md) for scaling guidance
