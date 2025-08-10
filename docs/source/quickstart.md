# Quick Start Guide

Get up and running with PYRVO2-RL in 5 minutes! This guide walks you through creating your first multi-agent simulation.

## Basic Concepts

PYRVO2-RL provides three main components:

1. **RVO2 Core** (`rvo2_rl.rvo2`): Low-level collision avoidance simulation
2. **RL Extensions** (`rvo2_rl.rl`): High-level RL-friendly interface 
3. **Utilities** (`rvo2_rl.util`): Helper functions and tools

## Your First Simulation

### Step 1: Import and Create Wrapper

```python
import rvo2_rl
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2

# Create an RL-optimized simulator
wrapper = RVO2RLWrapper(
    time_step=0.25,        # 250ms per simulation step
    neighbor_dist=15.0,    # Agents sense neighbors within 15 units
    max_neighbors=10,      # Observe up to 10 nearest neighbors
    radius=0.5,            # Agent radius for collision avoidance
    max_speed=2.0,         # Maximum agent speed
    mode=ObsMode.Cartesian # Use Cartesian coordinates for observations
)
```

### Step 2: Add Agents and Goals

```python
# Add agents at different starting positions
agent_positions = [
    Vector2(0.0, 0.0),     # Agent 0 at origin
    Vector2(5.0, 0.0),     # Agent 1 to the right
    Vector2(0.0, 5.0),     # Agent 2 above
    Vector2(5.0, 5.0)      # Agent 3 diagonally
]

for pos in agent_positions:
    wrapper.add_agent(pos)

# Set goals (agents will try to reach these positions)
goals = [
    Vector2(5.0, 5.0),     # Agent 0 moves diagonally
    Vector2(0.0, 5.0),     # Agent 1 moves left and up
    Vector2(5.0, 0.0),     # Agent 2 moves right and down  
    Vector2(0.0, 0.0)      # Agent 3 moves to origin
]

wrapper.set_goals(goals)

# Initialize the simulation (required after adding agents/goals)
wrapper.initialize()
```

### Step 3: Run Simulation and Get Observations

```python
import numpy as np

# Run simulation for 50 steps
for step in range(50):
    # Get observations for all agents
    observations = []
    for agent_id in range(4):  # We have 4 agents
        obs = wrapper.get_observation(agent_id)
        observations.append(obs)
    
    # Print agent positions every 10 steps
    if step % 10 == 0:
        agent_data = wrapper.get_agent_data_for_vis()
        print(f"Step {step}:")
        for data in agent_data:
            agent_id, x, y, velocity, pref_vel, dist_to_goal = data
            print(f"  Agent {agent_id}: pos=({x:.2f}, {y:.2f}), "
                  f"dist_to_goal={dist_to_goal:.2f}")
    
    # Compute preferred velocities (automatic collision avoidance)
    wrapper.set_preferred_velocities()
    
    # Advance simulation by one time step
    wrapper.get_simulator().do_step()

print("Simulation complete!")
```

## Understanding Observations

The observation vector contains multiple types of information:

```python
# Get observation bounds to understand the structure
bounds = wrapper.get_observation_bounds()
print("Observation structure:")
for i, description in enumerate(bounds["info"]):
    print(f"  {i}: {description}")

# Get a sample observation
obs = wrapper.get_observation(0)  # Observation for agent 0
print(f"Observation shape: {obs.shape}")
print(f"Observation values: {obs}")
```

## Adding LiDAR Sensing

Enable LiDAR for 360-degree distance sensing:

```python
# Create wrapper with LiDAR enabled
lidar_wrapper = RVO2RLWrapper(
    time_step=0.25,
    neighbor_dist=15.0,
    max_neighbors=5,
    radius=0.5,
    max_speed=2.0,
    mode=ObsMode.Cartesian,
    use_lidar=True,        # Enable LiDAR
    lidar_count=36,        # 36 rays (10-degree resolution)
    lidar_range=10.0       # Maximum sensing range
)

# Add a single agent
lidar_wrapper.add_agent(Vector2(0.0, 0.0))

# Create some obstacles for LiDAR to detect
sim = lidar_wrapper.get_simulator()

# Add a square obstacle
obstacle_vertices = [
    Vector2(3.0, 1.0),
    Vector2(4.0, 1.0), 
    Vector2(4.0, 2.0),
    Vector2(3.0, 2.0)
]
sim.add_obstacle(obstacle_vertices)
sim.process_obstacles()  # Required after adding obstacles

# Set goal and initialize
lidar_wrapper.set_goals([Vector2(5.0, 0.0)])
lidar_wrapper.initialize()

# Get LiDAR data
lidar_data = lidar_wrapper.get_lidar(0)
print(f"LiDAR data shape: {lidar_data.shape}")
print("Sample LiDAR readings (angle, distance):")
for i in range(0, len(lidar_data), 6):  # Show every 6th reading
    angle, distance = lidar_data[i]
    print(f"  {angle:.2f} rad, {distance:.3f} (normalized)")
```

## Working with Different Observation Modes

### Cartesian Mode (Default)

```python
cartesian_wrapper = RVO2RLWrapper(mode=ObsMode.Cartesian)
# Neighbor observations: [pos_x, pos_y, vel_x, vel_y, pref_vel_x, pref_vel_y]
```

### Polar Mode

```python
polar_wrapper = RVO2RLWrapper(mode=ObsMode.Polar)
# Neighbor observations: [pos_x, pos_y, vel_magnitude, vel_angle, pref_vel_mag, pref_vel_angle]
```

## Integration with RL Frameworks

### Gymnasium Environment Wrapper

Here's a basic Gymnasium environment wrapper:

```python
import gymnasium as gym
import numpy as np

class PYRVO2RLEnv(gym.Env):
    def __init__(self, num_agents=4):
        super().__init__()
        
        self.num_agents = num_agents
        self.wrapper = RVO2RLWrapper(
            time_step=0.25,
            neighbor_dist=15.0,
            max_neighbors=10,
            radius=0.5,
            max_speed=2.0,
            mode=ObsMode.Cartesian
        )
        
        # Define action and observation spaces
        # Actions: 2D velocity commands per agent
        self.action_space = gym.spaces.Box(
            low=-2.0, high=2.0, 
            shape=(num_agents, 2), 
            dtype=np.float32
        )
        
        # Observations: get bounds from wrapper
        self._setup_observation_space()
    
    def _setup_observation_space(self):
        # Add dummy agents to get observation bounds
        for i in range(self.num_agents):
            self.wrapper.add_agent(Vector2(0.0, 0.0))
        self.wrapper.set_goals([Vector2(1.0, 1.0)] * self.num_agents)
        self.wrapper.initialize()
        
        bounds = self.wrapper.get_observation_bounds()
        obs_low = np.array(bounds["low"], dtype=np.float32)
        obs_high = np.array(bounds["high"], dtype=np.float32)
        
        # Observation space for all agents
        self.observation_space = gym.spaces.Box(
            low=np.tile(obs_low, self.num_agents),
            high=np.tile(obs_high, self.num_agents),
            dtype=np.float32
        )
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # Reset wrapper (implementation needed)
        # For now, just get initial observations
        observations = []
        for agent_id in range(self.num_agents):
            obs = self.wrapper.get_observation(agent_id)
            observations.append(obs)
        
        return np.concatenate(observations), {}
    
    def step(self, actions):
        # Apply actions as preferred velocities
        for agent_id, action in enumerate(actions):
            velocity = Vector2(action[0], action[1])
            self.wrapper.set_preferred_velocity(agent_id, velocity)
        
        # Step simulation
        self.wrapper.get_simulator().do_step()
        
        # Get new observations
        observations = []
        for agent_id in range(self.num_agents):
            obs = self.wrapper.get_observation(agent_id)
            observations.append(obs)
        
        # Calculate rewards (distance to goal)
        rewards = []
        for agent_id in range(self.num_agents):
            dist = self.wrapper.get_distance_to_goal(agent_id, normalized=True)
            reward = -dist  # Negative distance as reward
            rewards.append(reward)
        
        # Check if done (all agents reached goals)
        dones = [dist < 0.1 for dist in 
                [self.wrapper.get_distance_to_goal(i) for i in range(self.num_agents)]]
        done = all(dones)
        
        return np.concatenate(observations), sum(rewards), done, False, {}

# Test the environment
env = PYRVO2RLEnv(num_agents=2)
obs, info = env.reset()
print(f"Initial observation shape: {obs.shape}")

# Random action
action = env.action_space.sample()
obs, reward, done, truncated, info = env.step(action)
print(f"After step - reward: {reward}, done: {done}")
```

## Next Steps

Now that you have the basics working:

1. **Explore Examples**: Check out [Usage Examples](examples/basic_usage.md) for more advanced scenarios
2. **Learn the API**: Read the [RL Extensions API](api/rl_extensions.md) for complete reference
3. **Optimize Performance**: See [Performance Tips](technical/performance.md) for large-scale simulations
4. **Customize Behavior**: Learn about different observation modes and sensing configurations

## Common Patterns

### Multi-Agent Training Loop

```python
def training_loop(env, agent_policy, num_episodes=1000):
    for episode in range(num_episodes):
        obs, info = env.reset()
        episode_reward = 0
        
        for step in range(200):  # Max episode length
            # Get actions from your policy
            actions = agent_policy.get_actions(obs)
            
            # Step environment
            obs, reward, done, truncated, info = env.step(actions)
            episode_reward += reward
            
            # Train your policy here
            agent_policy.update(obs, actions, reward, done)
            
            if done or truncated:
                break
        
        print(f"Episode {episode}: reward = {episode_reward}")
```

### Batch Observation Collection

```python
def collect_batch_observations(wrapper, num_agents):
    """Efficiently collect observations for all agents"""
    batch_obs = []
    for agent_id in range(num_agents):
        obs = wrapper.get_observation(agent_id)
        batch_obs.append(obs)
    return np.array(batch_obs)

# Use in your training loop
observations = collect_batch_observations(wrapper, 10)  # 10 agents
print(f"Batch shape: {observations.shape}")  # (10, obs_size)
```

You're now ready to build sophisticated multi-agent RL experiments with PYRVO2-RL!
