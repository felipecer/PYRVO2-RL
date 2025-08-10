# Basic Usage Examples

This section provides practical examples to get you started with PYRVO2-RL for common multi-agent scenarios.

## Example 1: Simple Two-Agent Swap

The most basic scenario: two agents exchanging positions.

```python
import rvo2_rl
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2        print(f"Step {step} - Agent 0 observation:")
        print(f"  Full observation shape: {obs.shape}")
        print(f"  First 10 values: {obs[:10]}")
        
        # Break down the observation components
        step_count = obs[0]
        dist_goal_x, dist_goal_y = obs[1], obs[2]
        
        print(f"  Step count: {step_count}")
        print(f"  Distance to goal: ({dist_goal_x:.3f}, {dist_goal_y:.3f})")
        
        # Get neighbor data separately for detailed analysis
        neighbors = wrapper.get_neighbors(0)
        print(f"  Neighbors array shape: {neighbors.shape}")
        
        # Check if masks are enabled by examining neighbor data structure
        has_mask = neighbors.shape[1] == 7  # 7 columns means mask is included
        
        print(f"  Neighbors (first 2 rows):")
        for i in range(min(2, neighbors.shape[0])):
            if has_mask:
                pos_x, pos_y, vel_x, vel_y, pv_x, pv_y, mask = neighbors[i]
                valid = "valid" if mask > 0.5 else "padded"
                print(f"    Neighbor {i}: pos=({pos_x:.2f}, {pos_y:.2f}), "
                      f"vel=({vel_x:.2f}, {vel_y:.2f}), mask={mask:.0f} ({valid})")
            else:
                pos_x, pos_y, vel_x, vel_y, pv_x, pv_y = neighbors[i]
                print(f"    Neighbor {i}: pos=({pos_x:.2f}, {pos_y:.2f}), "
                      f"vel=({vel_x:.2f}, {vel_y:.2f})")
        
        print()er
wrapper = RVO2RLWrapper(
    time_step=0.25,
    neighbor_dist=10.0,
    max_neighbors=5,
    radius=0.5,
    max_speed=1.5,
    mode=ObsMode.Cartesian
)

# Add two agents
agent1_id = wrapper.add_agent(Vector2(-5.0, 0.0))
agent2_id = wrapper.add_agent(Vector2(5.0, 0.0))

# Set goals (they want to swap positions)
wrapper.set_goals([Vector2(5.0, 0.0), Vector2(-5.0, 0.0)])
wrapper.initialize()

# Run simulation
for step in range(50):
    # Get observations
    obs1 = wrapper.get_observation(agent1_id)
    obs2 = wrapper.get_observation(agent2_id)
    
    print(f"Step {step}: Agent1 obs shape: {obs1.shape}, Agent2 obs shape: {obs2.shape}")
    
    # Use automatic goal-seeking behavior
    wrapper.set_preferred_velocities()
    wrapper.get_simulator().do_step()
    
    # Check distances to goals
    dist1 = wrapper.get_distance_to_goal(agent1_id)
    dist2 = wrapper.get_distance_to_goal(agent2_id)
    
    if step % 10 == 0:
        print(f"  Distances to goal: Agent1={dist1:.2f}, Agent2={dist2:.2f}")
    
    # Stop when both agents reach their goals
    if dist1 < 0.5 and dist2 < 0.5:
        print(f"Both agents reached goals at step {step}")
        break

print("Simple swap example completed!")
```

## Example 2: Four-Agent Circle Formation

Agents starting at corners of a square move to form a circle.

```python
import numpy as np
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2

def create_circle_formation():
    wrapper = RVO2RLWrapper(
        time_step=0.2,
        neighbor_dist=12.0,
        max_neighbors=8,
        radius=0.4,
        max_speed=2.0,
        mode=ObsMode.Cartesian
    )
    
    # Starting positions: corners of a square
    start_positions = [
        Vector2(-3.0, -3.0),  # Bottom-left
        Vector2(3.0, -3.0),   # Bottom-right
        Vector2(3.0, 3.0),    # Top-right
        Vector2(-3.0, 3.0)    # Top-left
    ]
    
    # Goal positions: points on a circle
    radius = 4.0
    goal_positions = []
    for i in range(4):
        angle = i * 2 * np.pi / 4 + np.pi/4  # Offset by 45 degrees
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        goal_positions.append(Vector2(x, y))
    
    # Add agents
    agent_ids = []
    for pos in start_positions:
        agent_id = wrapper.add_agent(pos)
        agent_ids.append(agent_id)
    
    # Set goals
    wrapper.set_goals(goal_positions)
    wrapper.initialize()
    
    return wrapper, agent_ids

# Run the simulation
wrapper, agent_ids = create_circle_formation()

print("Four-agent circle formation:")
print("Starting simulation...")

for step in range(100):
    # Collect all observations
    observations = []
    for agent_id in agent_ids:
        obs = wrapper.get_observation(agent_id)
        observations.append(obs)
    
    # Print progress every 20 steps
    if step % 20 == 0:
        print(f"\\nStep {step}:")
        for i, agent_id in enumerate(agent_ids):
            pos = wrapper.get_simulator().get_agent_position(agent_id)
            dist = wrapper.get_distance_to_goal(agent_id)
            print(f"  Agent {i}: pos=({pos.x():.2f}, {pos.y():.2f}), dist_to_goal={dist:.2f}")
    
    # Use automatic preferred velocities
    wrapper.set_preferred_velocities()
    wrapper.get_simulator().do_step()
    
    # Check if all agents reached their goals
    max_distance = max(wrapper.get_distance_to_goal(agent_id) for agent_id in agent_ids)
    if max_distance < 0.3:
        print(f"\\nAll agents reached circle formation at step {step}")
        break

print("Circle formation example completed!")
```

## Example 3: RL Action Integration

Demonstrate how to apply reinforcement learning actions to agents.

```python
import numpy as np
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2

class SimpleRLController:
    """Simple RL-style controller that moves agents toward goals."""
    
    def __init__(self, max_speed=1.0):
        self.max_speed = max_speed
    
    def get_action(self, observation, goal_position, current_position):
        """
        Extract action from observation and goal information.
        
        Args:
            observation: Agent's observation vector
            goal_position: Goal as (x, y) tuple
            current_position: Current position as (x, y) tuple
        
        Returns:
            Action as (vx, vy) tuple
        """
        # Simple goal-seeking policy
        dx = goal_position[0] - current_position[0]
        dy = goal_position[1] - current_position[1]
        
        # Normalize to max speed
        distance = np.sqrt(dx**2 + dy**2)
        if distance > 0.1:  # Not at goal yet
            vx = (dx / distance) * self.max_speed
            vy = (dy / distance) * self.max_speed
        else:
            vx, vy = 0.0, 0.0
        
        return (vx, vy)

def rl_action_example():
    """Example of using RL-style actions with PYRVO2-RL."""
    
    # Create simulation
    wrapper = RVO2RLWrapper(
        time_step=0.15,
        neighbor_dist=8.0,
        max_neighbors=6,
        radius=0.3,
        max_speed=1.5,
        mode=ObsMode.Cartesian
    )
    
    # Add agents in a line
    num_agents = 6
    agent_ids = []
    start_positions = []
    goal_positions = []
    
    for i in range(num_agents):
        # Start positions: horizontal line
        start_x = i * 2.0 - 5.0  # Range from -5 to 5
        start_y = 0.0
        start_pos = Vector2(start_x, start_y)
        start_positions.append((start_x, start_y))
        
        # Goal positions: vertical line (rotated 90 degrees)
        goal_x = 0.0
        goal_y = i * 1.5 - 3.75  # Centered around origin
        goal_pos = Vector2(goal_x, goal_y)
        goal_positions.append((goal_x, goal_y))
        
        agent_id = wrapper.add_agent(start_pos)
        agent_ids.append(agent_id)
    
    wrapper.set_goals([Vector2(gx, gy) for gx, gy in goal_positions])
    wrapper.initialize()
    
    # Create RL controller
    controller = SimpleRLController(max_speed=1.2)
    
    print("RL Action Integration Example:")
    print("Agents moving from horizontal to vertical formation")
    
    for step in range(80):
        # Get observations and apply RL actions
        for i, agent_id in enumerate(agent_ids):
            # Get observation
            obs = wrapper.get_observation(agent_id)
            
            # Get current position
            current_pos = wrapper.get_simulator().get_agent_position(agent_id)
            current_position = (current_pos.x(), current_pos.y())
            
            # Get action from RL controller
            action = controller.get_action(obs, goal_positions[i], current_position)
            
            # Apply action as preferred velocity
            preferred_vel = Vector2(action[0], action[1])
            wrapper.set_preferred_velocity(agent_id, preferred_vel)
        
        # Step simulation (this computes actual velocities using ORCA)
        wrapper.get_simulator().do_step()
        
        # Print progress
        if step % 15 == 0:
            print(f"\\nStep {step}:")
            avg_distance = np.mean([wrapper.get_distance_to_goal(agent_id) 
                                   for agent_id in agent_ids])
            print(f"  Average distance to goal: {avg_distance:.2f}")
            
            # Show first agent details
            pos = wrapper.get_simulator().get_agent_position(agent_ids[0])
            vel = wrapper.get_simulator().get_agent_velocity(agent_ids[0])
            print(f"  Agent 0: pos=({pos.x():.2f}, {pos.y():.2f}), "
                  f"vel=({vel.x():.2f}, {vel.y():.2f})")
    
    print("\\nRL action integration example completed!")

# Run the example
rl_action_example()
```

## Example 4: Observation Analysis

Understanding what information is available in observations.

```python
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2

def analyze_observations():
    """Detailed analysis of observation structure and content."""
    
    # Create wrapper with detailed observation settings
    wrapper = RVO2RLWrapper(
        time_step=0.2,
        neighbor_dist=6.0,
        max_neighbors=4,
        radius=0.4,
        max_speed=1.0,
        mode=ObsMode.Cartesian,
        use_obs_mask=True  # Include validity masks
    )
    
    # Add multiple agents for neighbor observations
    positions = [
        Vector2(0.0, 0.0),    # Center agent (we'll analyze this one)
        Vector2(2.0, 0.0),    # Right neighbor
        Vector2(-1.5, 1.5),   # Top-left neighbor
        Vector2(0.0, -2.5)    # Bottom neighbor (farther away)
    ]
    
    agent_ids = []
    for pos in positions:
        agent_id = wrapper.add_agent(pos)
        agent_ids.append(agent_id)
    
    # Set goals
    goals = [Vector2(5.0, 5.0), Vector2(-3.0, 2.0), Vector2(3.0, -2.0), Vector2(-1.0, 4.0)]
    wrapper.set_goals(goals)
    wrapper.initialize()
    
    print("Observation Analysis Example")
    print("=" * 40)
    
    # Get observation structure information
    bounds = wrapper.get_observation_limits()
    
    print(f"Observation mode: {bounds['mode']}")
    print(f"Observation size: {len(bounds['low'])}")
    print(f"Bounds shape: low={len(bounds['low'])}, high={len(bounds['high'])}")
    print()
    
    print("Observation structure:")
    for i, description in enumerate(bounds['info']):
        print(f"  [{i:2d}] {description}")
    print()
    
    # Run a few steps and analyze observations
    for step in range(5):
        wrapper.set_preferred_velocities()
        wrapper.get_simulator().do_step()
        
        # Analyze observation for center agent (agent 0)
        obs = wrapper.get_observation(0)
        
        print(f"Step {step} - Agent 0 observation:")
        print(f"  Full observation shape: {obs.shape}")
        print(f"  First 10 values: {obs[:10]}")
        
        # Check if masks are enabled by examining neighbor data structure
        neighbors = wrapper.get_neighbors(0)
        has_mask = neighbors.shape[1] == 7  # 7 columns means mask is included
        
        print(f"  Neighbors (first 2 rows):")
        for i in range(min(2, neighbors.shape[0])):
            if has_mask:
                pos_x, pos_y, vel_x, vel_y, pv_x, pv_y, mask = neighbors[i]
                valid = "valid" if mask > 0.5 else "padded"
                print(f"    Neighbor {i}: pos=({pos_x:.2f}, {pos_y:.2f}), "
                      f"vel=({vel_x:.2f}, {vel_y:.2f}), mask={mask:.0f} ({valid})")
            else:
                pos_x, pos_y, vel_x, vel_y, pv_x, pv_y = neighbors[i]
                print(f"    Neighbor {i}: pos=({pos_x:.2f}, {pos_y:.2f}), "
                      f"vel=({vel_x:.2f}, {vel_y:.2f})")
        
        print()

# Run the analysis
analyze_observations()
```

## Example 5: Custom Environment Wrapper

Create a Gymnasium-compatible environment for RL training.

```python
import gymnasium as gym
import numpy as np
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2

class MultiAgentNavigationEnv(gym.Env):
    """
    Custom Gymnasium environment for multi-agent navigation.
    
    Observation space: Concatenated observations from all agents
    Action space: 2D velocity commands for each agent
    Reward: Based on progress toward goals and collision avoidance
    """
    
    def __init__(self, num_agents=4, max_episode_steps=200):
        super().__init__()
        
        self.num_agents = num_agents
        self.max_episode_steps = max_episode_steps
        self.current_step = 0
        
        # Create RVO2-RL wrapper
        self.wrapper = RVO2RLWrapper(
            time_step=0.2,
            neighbor_dist=8.0,
            max_neighbors=6,
            radius=0.4,
            max_speed=1.5,
            mode=ObsMode.Cartesian,
            use_obs_mask=True
        )
        
        # Initialize environment
        self._setup_agents()
        self._setup_spaces()
    
    def _setup_agents(self):
        """Add agents to the simulation."""
        # Add agents in random positions
        self.start_positions = []
        self.goal_positions = []
        
        np.random.seed(42)  # For reproducible initialization
        
        for i in range(self.num_agents):
            # Random start position
            start_x = np.random.uniform(-8, 8)
            start_y = np.random.uniform(-8, 8)
            start_pos = Vector2(start_x, start_y)
            self.start_positions.append(start_pos)
            
            # Random goal position (far from start)
            while True:
                goal_x = np.random.uniform(-8, 8)
                goal_y = np.random.uniform(-8, 8)
                distance = np.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
                if distance > 5.0:  # Ensure goal is far enough
                    break
            goal_pos = Vector2(goal_x, goal_y)
            self.goal_positions.append(goal_pos)
            
            self.wrapper.add_agent(start_pos)
        
        self.wrapper.set_goals(self.goal_positions)
        self.wrapper.initialize()
    
    def _setup_spaces(self):
        """Define observation and action spaces."""
        # Get observation bounds from wrapper
        bounds = self.wrapper.get_observation_limits()
        obs_low = np.array(bounds["low"], dtype=np.float32)
        obs_high = np.array(bounds["high"], dtype=np.float32)
        
        # Observation space: concatenated observations from all agents
        self.observation_space = gym.spaces.Box(
            low=np.tile(obs_low, self.num_agents),
            high=np.tile(obs_high, self.num_agents),
            dtype=np.float32
        )
        
        # Action space: 2D velocity for each agent
        self.action_space = gym.spaces.Box(
            low=-1.5, high=1.5,
            shape=(self.num_agents, 2),
            dtype=np.float32
        )
    
    def reset(self, seed=None, options=None):
        """Reset environment to initial state."""
        super().reset(seed=seed)
        
        self.current_step = 0
        
        # Reset agent positions and goals
        self.wrapper.reset_position_and_goals_to_init()
        
        # Get initial observations
        observations = self._get_observations()
        
        return observations, {}
    
    def step(self, actions):
        """Execute one step of the environment."""
        # Apply actions as preferred velocities
        for i, action in enumerate(actions):
            velocity = Vector2(float(action[0]), float(action[1]))
            self.wrapper.set_preferred_velocity(i, velocity)
        
        # Step simulation
        self.wrapper.get_simulator().do_step()
        self.current_step += 1
        
        # Get new observations
        observations = self._get_observations()
        
        # Calculate rewards
        rewards = self._calculate_rewards()
        
        # Check termination conditions
        terminated = self._check_terminated()
        truncated = self.current_step >= self.max_episode_steps
        
        info = {
            'distances_to_goals': [self.wrapper.get_distance_to_goal(i) 
                                 for i in range(self.num_agents)],
            'step': self.current_step
        }
        
        return observations, rewards, terminated, truncated, info
    
    def _get_observations(self):
        """Get concatenated observations from all agents."""
        observations = []
        for i in range(self.num_agents):
            obs = self.wrapper.get_observation(i)
            observations.append(obs)
        return np.concatenate(observations)
    
    def _calculate_rewards(self):
        """Calculate rewards based on progress and behavior."""
        total_reward = 0
        
        for i in range(self.num_agents):
            # Distance-based reward
            distance = self.wrapper.get_distance_to_goal(i)
            distance_reward = -distance * 0.1
            
            # Goal reaching bonus
            goal_bonus = 10.0 if distance < 0.5 else 0.0
            
            # Speed penalty (encourage efficiency)
            velocity = self.wrapper.get_simulator().get_agent_velocity(i)
            speed = np.sqrt(velocity.x()**2 + velocity.y()**2)
            speed_penalty = -0.01 * speed if distance > 0.5 else 0.0
            
            agent_reward = distance_reward + goal_bonus + speed_penalty
            total_reward += agent_reward
        
        return total_reward
    
    def _check_terminated(self):
        """Check if episode should terminate (all agents reached goals)."""
        for i in range(self.num_agents):
            distance = self.wrapper.get_distance_to_goal(i)
            if distance > 0.5:  # Not all agents at goals
                return False
        return True
    
    def render(self, mode='human'):
        """Render the environment (optional)."""
        if mode == 'human':
            print(f"Step {self.current_step}:")
            for i in range(self.num_agents):
                pos = self.wrapper.get_simulator().get_agent_position(i)
                distance = self.wrapper.get_distance_to_goal(i)
                print(f"  Agent {i}: pos=({pos.x():.2f}, {pos.y():.2f}), "
                      f"dist_to_goal={distance:.2f}")

# Example usage of the custom environment
def test_custom_environment():
    """Test the custom Gymnasium environment."""
    
    env = MultiAgentNavigationEnv(num_agents=3, max_episode_steps=150)
    
    print("Testing Custom Environment")
    print(f"Observation space: {env.observation_space.shape}")
    print(f"Action space: {env.action_space.shape}")
    print()
    
    # Run a simple episode with random actions
    obs, info = env.reset()
    print(f"Initial observation shape: {obs.shape}")
    
    total_reward = 0
    
    for step in range(50):
        # Random actions for demonstration
        actions = env.action_space.sample()
        
        obs, reward, terminated, truncated, info = env.step(actions)
        total_reward += reward
        
        if step % 10 == 0:
            print(f"Step {step}: reward={reward:.2f}, total_reward={total_reward:.2f}")
            avg_distance = np.mean(info['distances_to_goals'])
            print(f"  Average distance to goals: {avg_distance:.2f}")
        
        if terminated or truncated:
            print(f"Episode ended at step {step}")
            print(f"  Terminated: {terminated}, Truncated: {truncated}")
            break
    
    print(f"Final total reward: {total_reward:.2f}")

# Run the test
test_custom_environment()
```

## Key Takeaways

1. **Start Simple**: Begin with basic two-agent scenarios before scaling up
2. **Observation Structure**: Use `get_observation_limits()` to understand the observation layout
3. **Action Integration**: Apply RL actions via `set_preferred_velocity()` 
4. **Environment Wrapping**: Create Gymnasium environments for standard RL training
5. **Parameter Tuning**: Adjust `neighbor_dist`, `max_neighbors`, and `time_step` for your use case

## Next Steps

- Explore [Multi-Agent Scenarios](multi_agent.md) for complex formations and behaviors
- See [Advanced Examples](advanced.md) for LiDAR integration and obstacle navigation
- Check [Performance Tips](../technical/performance.md) for optimizing large simulations
