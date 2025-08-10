# RL Extensions API Reference

The `rvo2_rl.rl` module provides reinforcement learning optimized interfaces for multi-agent collision avoidance simulation.

## Core Classes

### RVO2RLWrapper

The main class for RL-enabled multi-agent simulation.

```python
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2

wrapper = RVO2RLWrapper(
    time_step=0.25,
    neighbor_dist=15.0,
    max_neighbors=10,
    time_horizon=5.0,
    time_horizon_obst=5.0,
    radius=0.5,
    max_speed=2.0,
    velocity=Vector2(),
    mode=ObsMode.Cartesian,
    use_obs_mask=False,
    use_lidar=False,
    lidar_count=360,
    lidar_range=18.0,
    max_step_count=256
)
```

#### Constructor Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `time_step` | `float` | `0.25` | Simulation time step in seconds |
| `neighbor_dist` | `float` | `15.0` | Maximum distance for neighbor detection |
| `max_neighbors` | `int` | `10` | Maximum number of neighbors to observe |
| `time_horizon` | `float` | `5.0` | Time horizon for velocity planning |
| `time_horizon_obst` | `float` | `5.0` | Time horizon for obstacle avoidance |
| `radius` | `float` | `0.5` | Agent radius for collision detection |
| `max_speed` | `float` | `2.0` | Maximum agent speed |
| `velocity` | `Vector2` | `Vector2()` | Initial velocity for new agents |
| `mode` | `ObsMode` | `ObsMode.Cartesian` | Observation coordinate system |
| `use_obs_mask` | `bool` | `False` | Include validity masks in observations |
| `use_lidar` | `bool` | `False` | Enable LiDAR-style distance sensing |
| `lidar_count` | `int` | `360` | Number of LiDAR rays (angular resolution) |
| `lidar_range` | `float` | `18.0` | Maximum LiDAR sensing range |
| `max_step_count` | `int` | `256` | Maximum episode length |

#### Core Methods

##### Initialization and Setup

```python
def initialize(self) -> None:
    """Initialize the simulation after adding agents and setting goals.
    
    Raises:
        RuntimeError: If simulator is not properly configured
        RuntimeError: If no agents have been added
        RuntimeError: If goals are not set for all agents
        RuntimeError: If LIDAR is enabled but not properly initialized
    """
```

```python  
def add_agent(self, position: Vector2) -> int:
    """Add an agent at the specified position with default parameters.
    
    Args:
        position: Initial position as Vector2
        
    Returns:
        Agent ID (integer starting from 0)
    """

def add_agent(self, position: Vector2, neighbor_dist: float, 
              max_neighbors: int, time_horizon: float,
              time_horizon_obst: float, radius: float, 
              max_speed: float, velocity: Vector2 = Vector2()) -> int:
    """Add an agent with custom parameters.
    
    Args:
        position: Initial position
        neighbor_dist: Neighbor detection range for this agent
        max_neighbors: Max neighbors for this agent
        time_horizon: Planning horizon for this agent
        time_horizon_obst: Obstacle avoidance horizon for this agent
        radius: Collision radius for this agent
        max_speed: Maximum speed for this agent
        velocity: Initial velocity for this agent
        
    Returns:
        Agent ID
    """
```

##### Goal Management

```python
def set_goals(self, goals: List[Vector2]) -> None:
    """Set goals for all agents.
    
    Args:
        goals: List of goal positions, one per agent
        
    Raises:
        ValueError: If number of goals doesn't match number of agents
    """

def set_goal(self, agent_id: int, goal: Vector2) -> None:
    """Set goal for a specific agent.
    
    Args:
        agent_id: Agent identifier
        goal: Goal position
        
    Raises:
        IndexError: If agent_id is invalid
    """

def get_goal(self, agent_id: int) -> Tuple[float, float]:
    """Get goal position for a specific agent.
    
    Args:
        agent_id: Agent identifier
        
    Returns:
        Goal position as (x, y) tuple
        
    Raises:
        IndexError: If agent_id is invalid
    """

def get_goals(self) -> np.ndarray:
    """Get all agent goals as a NumPy array.
    
    Returns:
        Array of shape (num_agents, 2) with goal positions
    """
```

##### Observation Methods

```python
def get_observation(self, agent_id: int) -> np.ndarray:
    """Get complete observation vector for an agent.
    
    The observation concatenates in order:
    - step_count (1 value)
    - distance_to_goal_x, distance_to_goal_y (2 values)
    - lidar_data (if enabled: lidar_count * 2 or 3 values)
    - neighbor_data (max_neighbors * 6 or 7 values)
    
    Args:
        agent_id: Agent identifier
        
    Returns:
        Flattened observation array
        
    Raises:
        IndexError: If agent_id is invalid
        RuntimeError: If simulation not initialized
    """

def get_observation_limits(self) -> Dict[str, Any]:
    """Get observation space bounds and structure information.
    
    Returns:
        Dictionary with keys:
        - 'mode': 'cartesian' or 'polar'
        - 'low': NumPy array of lower bounds
        - 'high': NumPy array of upper bounds  
        - 'info': List of strings describing each observation component
    """

def get_neighbors(self, agent_id: int) -> np.ndarray:
    """Get neighbor observations for an agent.
    
    Returns neighbor data in the format specified by the observation mode:
    
    Cartesian mode:
    - Without mask: (max_neighbors, 6) array with columns [pos_x, pos_y, vel_x, vel_y, pref_vel_x, pref_vel_y]
    - With mask: (max_neighbors, 7) array with additional mask column
    
    Polar mode:
    - Without mask: (max_neighbors, 6) array with columns [pos_x, pos_y, vel_mag, vel_angle, pref_vel_mag, pref_vel_angle]
    - With mask: (max_neighbors, 7) array with additional mask column
    
    Args:
        agent_id: Agent identifier
        
    Returns:
        Neighbor observation array
        
    Raises:
        IndexError: If agent_id is invalid
    """

def get_lidar(self, agent_id: int) -> np.ndarray:
    """Get LiDAR observations for an agent.
    
    Returns LiDAR data as an array where each row represents one ray:
    - Without mask: (lidar_count, 2) array with columns [angle, normalized_distance]
    - With mask: (lidar_count, 3) array with columns [angle, normalized_distance, hit_mask]
    
    Args:
        agent_id: Agent identifier
        
    Returns:
        LiDAR observation array
        
    Raises:
        IndexError: If agent_id is invalid
        RuntimeError: If LiDAR not enabled
    """
```

##### Distance and Goal Utilities

```python
def get_distance_to_goal(self, agent_id: int, normalized: bool = False) -> float:
    """Get distance from agent to its goal.
    
    Args:
        agent_id: Agent identifier
        normalized: If True, normalize by initial distance to goal
        
    Returns:
        Distance to goal
        
    Raises:
        IndexError: If agent_id is invalid
    """

def get_all_distances_to_goals(self, normalized: bool = False) -> List[float]:
    """Get distances from all agents to their goals.
    
    Args:
        normalized: If True, normalize by initial distances
        
    Returns:
        List of distances, one per agent
    """
```

##### Simulation Control

```python
def set_preferred_velocity(self, agent_id: int, velocity: Vector2) -> None:
    """Set preferred velocity for a specific agent.
    
    This is typically used to apply RL actions to agents.
    
    Args:
        agent_id: Agent identifier
        velocity: Preferred velocity vector
        
    Raises:
        IndexError: If agent_id is invalid
    """

def set_preferred_velocities(self) -> None:
    """Compute and set preferred velocities for all agents.
    
    This uses the internal goal-seeking behavior to compute
    preferred velocities automatically.
    """

def get_simulator(self) -> RVOSimulator:
    """Get access to the underlying RVO2 simulator.
    
    Returns:
        RVOSimulator instance for direct low-level access
    """

def do_step(self) -> None:
    """Advance simulation by one time step.
    
    This is equivalent to calling get_simulator().do_step()
    """
```

##### Batch Data Collection

```python
def get_agent_data_for_vis(self) -> List[Tuple]:
    """Get batch data for all agents for visualization.
    
    Returns:
        List of tuples, each containing:
        (agent_id, pos_x, pos_y, (vel_x, vel_y), (pref_vel_x, pref_vel_y), dist_to_goal)
    """
```

##### Episode Management

```python
def get_step_count(self) -> int:
    """Get current simulation step count.
    
    Returns:
        Number of steps since last reset
    """

def reset_step_count(self) -> None:
    """Reset step counter to zero."""

def set_current_goals_as_initial_goals(self) -> None:
    """Store current goals as initial goals for reset purposes."""

def set_current_positions_as_initial_positions(self) -> None:
    """Store current positions as initial positions for reset purposes."""

def reset_position_and_goals_to_init(self) -> None:
    """Reset all agents to their initial positions and goals."""
```

## Enumerations

### ObsMode

Defines the coordinate system for neighbor observations.

```python
from rvo2_rl.rl import ObsMode

# Available modes
ObsMode.Cartesian  # Use Cartesian coordinates (x, y, vx, vy)
ObsMode.Polar      # Use polar coordinates (r, θ, v_mag, v_angle)
```

#### Cartesian Mode (`ObsMode.Cartesian`)

Neighbor observations in Cartesian coordinates:

| Column | Description | Range |
|--------|-------------|-------|
| 0 | `pos_x` | Relative x position | `[-neighbor_dist, neighbor_dist]` |
| 1 | `pos_y` | Relative y position | `[-neighbor_dist, neighbor_dist]` |
| 2 | `vel_x` | X velocity component | `[-max_speed, max_speed]` |
| 3 | `vel_y` | Y velocity component | `[-max_speed, max_speed]` |
| 4 | `pref_vel_x` | Preferred X velocity | `[-max_speed, max_speed]` |
| 5 | `pref_vel_y` | Preferred Y velocity | `[-max_speed, max_speed]` |
| 6 | `mask` (optional) | Validity mask | `{0.0, 1.0}` |

#### Polar Mode (`ObsMode.Polar`)

Neighbor observations in polar coordinates:

| Column | Description | Range |
|--------|-------------|-------|
| 0 | `pos_x` | Relative x position | `[-neighbor_dist, neighbor_dist]` |
| 1 | `pos_y` | Relative y position | `[-neighbor_dist, neighbor_dist]` |
| 2 | `vel_mag` | Velocity magnitude | `[0, max_speed]` |
| 3 | `vel_angle` | Velocity direction (radians) | `[-π, π]` |
| 4 | `pref_vel_mag` | Preferred velocity magnitude | `[0, max_speed]` |
| 5 | `pref_vel_angle` | Preferred velocity direction | `[-π, π]` |
| 6 | `mask` (optional) | Validity mask | `{0.0, 1.0}` |

## Data Structures

### BatchAgentData

Structure returned by `get_agent_data_for_vis()`:

```python
@dataclass
class BatchAgentData:
    id: int           # Agent identifier
    px: float         # Position X
    py: float         # Position Y  
    vx: float         # Velocity X
    vy: float         # Velocity Y
    pvx: float        # Preferred velocity X
    pvy: float        # Preferred velocity Y
    dist_goal: float  # Distance to goal
```

## Observation Space Structure

The complete observation vector is structured as follows:

```
[step_count,                          # 1 value
 dist_to_goal_x, dist_to_goal_y,      # 2 values
 lidar_angles[], lidar_ranges[],      # lidar_count * 2 values (if enabled)
 lidar_masks[],                       # lidar_count values (if mask enabled)
 neighbor_data_flattened[]]           # max_neighbors * (6 or 7) values
```

### Observation Bounds

Use `get_observation_limits()` to get the exact structure:

```python
bounds = wrapper.get_observation_limits()

print("Observation mode:", bounds["mode"])
print("Observation size:", len(bounds["low"]))
print("Component descriptions:")
for i, desc in enumerate(bounds["info"]):
    print(f"  [{i:2d}] {desc}")

# Use bounds for Gymnasium spaces
import gymnasium as gym
observation_space = gym.spaces.Box(
    low=bounds["low"],
    high=bounds["high"],
    dtype=np.float32
)
```

## Usage Examples

### Basic Multi-Agent Setup

```python
import numpy as np
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2

# Create wrapper
wrapper = RVO2RLWrapper(
    time_step=0.1,
    neighbor_dist=10.0,
    max_neighbors=5,
    radius=0.5,
    max_speed=1.5,
    mode=ObsMode.Cartesian
)

# Add agents in a circle
num_agents = 8
radius = 5.0
for i in range(num_agents):
    angle = 2 * np.pi * i / num_agents
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    wrapper.add_agent(Vector2(x, y))

# Set goals opposite to starting positions
goals = []
for i in range(num_agents):
    angle = 2 * np.pi * i / num_agents + np.pi  # Opposite side
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    goals.append(Vector2(x, y))

wrapper.set_goals(goals)
wrapper.initialize()

# Simulation loop
for step in range(100):
    # Collect observations
    observations = []
    for agent_id in range(num_agents):
        obs = wrapper.get_observation(agent_id)
        observations.append(obs)
    
    # Apply your RL policy here
    # For demo, use automatic goal-seeking
    wrapper.set_preferred_velocities()
    
    # Step simulation
    wrapper.do_step()
    
    # Check progress
    distances = wrapper.get_all_distances_to_goals()
    if max(distances) < 0.5:  # All agents close to goals
        print(f"All agents reached goals in {step} steps!")
        break
```

### LiDAR-Enabled Simulation

```python
# Create wrapper with LiDAR
lidar_wrapper = RVO2RLWrapper(
    time_step=0.1,
    neighbor_dist=8.0,
    max_neighbors=3,
    radius=0.3,
    max_speed=1.0,
    mode=ObsMode.Polar,
    use_lidar=True,
    lidar_count=72,      # 5-degree resolution
    lidar_range=6.0,
    use_obs_mask=True    # Include validity masks
)

# Add agent
agent_id = lidar_wrapper.add_agent(Vector2(0.0, 0.0))

# Add obstacles
sim = lidar_wrapper.get_simulator()
obstacles = [
    [Vector2(2.0, -1.0), Vector2(3.0, -1.0), Vector2(3.0, 1.0), Vector2(2.0, 1.0)],  # Wall
    [Vector2(-1.0, 2.0), Vector2(1.0, 2.0), Vector2(1.0, 3.0), Vector2(-1.0, 3.0)]   # Wall
]
for obstacle in obstacles:
    sim.add_obstacle(obstacle)
sim.process_obstacles()

# Set goal and initialize
lidar_wrapper.set_goals([Vector2(4.0, 4.0)])
lidar_wrapper.initialize()

# Get LiDAR data
lidar_data = lidar_wrapper.get_lidar(agent_id)
print(f"LiDAR shape: {lidar_data.shape}")  # (72, 3) with mask
print("Sample readings:")
for i in range(0, 72, 12):  # Every 60 degrees
    angle, distance, mask = lidar_data[i]
    print(f"  {np.degrees(angle):6.1f}°: {distance:.3f} ({'hit' if mask else 'miss'})")
```

### Custom RL Integration

```python
class MultiAgentRLEnv:
    def __init__(self, num_agents=5):
        self.num_agents = num_agents
        self.wrapper = RVO2RLWrapper(
            time_step=0.2,
            neighbor_dist=12.0,
            max_neighbors=4,
            radius=0.4,
            max_speed=1.2,
            mode=ObsMode.Cartesian,
            max_step_count=500
        )
        self._setup_agents()
    
    def _setup_agents(self):
        # Add agents randomly
        for _ in range(self.num_agents):
            x = np.random.uniform(-8, 8)
            y = np.random.uniform(-8, 8)
            self.wrapper.add_agent(Vector2(x, y))
        
        # Set random goals
        goals = []
        for _ in range(self.num_agents):
            x = np.random.uniform(-8, 8)
            y = np.random.uniform(-8, 8)
            goals.append(Vector2(x, y))
        
        self.wrapper.set_goals(goals)
        self.wrapper.initialize()
    
    def step(self, actions):
        # Apply actions as preferred velocities
        for agent_id, action in enumerate(actions):
            velocity = Vector2(action[0], action[1])
            self.wrapper.set_preferred_velocity(agent_id, velocity)
        
        # Step simulation
        self.wrapper.do_step()
        
        # Get observations and rewards
        observations = []
        rewards = []
        for agent_id in range(self.num_agents):
            obs = self.wrapper.get_observation(agent_id)
            observations.append(obs)
            
            # Reward based on progress toward goal
            dist = self.wrapper.get_distance_to_goal(agent_id, normalized=True)
            reward = -dist + (1.0 if dist < 0.1 else 0.0)  # Bonus for reaching goal
            rewards.append(reward)
        
        # Check if episode is done
        step_count = self.wrapper.get_step_count()
        max_steps = 500
        done = step_count >= max_steps
        
        return observations, rewards, done
    
    def reset(self):
        # Reset to initial positions and goals
        self.wrapper.reset_position_and_goals_to_init()
        self.wrapper.reset_step_count()
        
        # Get initial observations
        observations = []
        for agent_id in range(self.num_agents):
            obs = self.wrapper.get_observation(agent_id)
            observations.append(obs)
        
        return observations

# Usage
env = MultiAgentRLEnv(num_agents=6)
observations = env.reset()

for episode_step in range(100):
    # Random actions for demo
    actions = np.random.uniform(-1.0, 1.0, (env.num_agents, 2))
    
    observations, rewards, done = env.step(actions)
    
    if done:
        print(f"Episode finished after {episode_step + 1} steps")
        break
```

This comprehensive API reference provides all the tools needed for sophisticated multi-agent RL experiments with collision avoidance.
