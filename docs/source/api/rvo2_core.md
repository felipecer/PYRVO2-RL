# RVO2 Core API Reference

The `rvo2_rl.rvo2` module provides direct access to the underlying RVO2 collision avoidance simulation engine.

## Overview

The RVO2 core implements the Optimal Reciprocal Collision Avoidance (ORCA) algorithm for multi-agent local navigation. This low-level API gives you direct control over the simulation, while the [RL Extensions](rl_extensions.md) provide higher-level abstractions for reinforcement learning.

## Core Classes

### RVOSimulator

The main simulation engine for multi-agent collision avoidance.

```python
from rvo2_rl.rvo2 import RVOSimulator, Vector2

# Create simulator with default parameters
sim = RVOSimulator()

# Create simulator with custom global parameters
sim = RVOSimulator(
    timeStep=0.25,          # Time step in seconds
    neighborDist=15.0,      # Default neighbor detection range
    maxNeighbors=10,        # Default max neighbors per agent
    timeHorizon=5.0,        # Default planning time horizon
    timeHorizonObst=5.0,    # Default obstacle avoidance horizon
    radius=0.5,             # Default agent radius
    maxSpeed=2.0,           # Default maximum speed
    velocity=Vector2(0, 0)  # Default initial velocity
)
```

#### Agent Management

##### Adding Agents

```python
def add_agent(position: Vector2) -> int:
    """Add an agent with default parameters.
    
    Args:
        position: Initial position of the agent
        
    Returns:
        Agent ID (starting from 0)
    """

def add_agent(position: Vector2, neighbor_dist: float, max_neighbors: int,
              time_horizon: float, time_horizon_obst: float, 
              radius: float, max_speed: float, 
              velocity: Vector2 = Vector2()) -> int:
    """Add an agent with custom parameters.
    
    Args:
        position: Initial position
        neighbor_dist: Maximum distance to consider other agents
        max_neighbors: Maximum number of neighbors to consider
        time_horizon: Time horizon for agent-agent collision avoidance
        time_horizon_obst: Time horizon for agent-obstacle collision avoidance
        radius: Agent radius for collision detection
        max_speed: Maximum agent speed
        velocity: Initial velocity
        
    Returns:
        Agent ID
    """
```

##### Agent Information Getters

```python
def get_num_agents() -> int:
    """Get the total number of agents in the simulation."""

def get_agent_position(agent_id: int) -> Vector2:
    """Get the current position of an agent."""

def get_agent_velocity(agent_id: int) -> Vector2:
    """Get the current velocity of an agent."""

def get_agent_pref_velocity(agent_id: int) -> Vector2:
    """Get the preferred velocity of an agent."""

def get_agent_radius(agent_id: int) -> float:
    """Get the radius of an agent."""

def get_agent_max_speed(agent_id: int) -> float:
    """Get the maximum speed of an agent."""

def get_agent_neighbor_dist(agent_id: int) -> float:
    """Get the neighbor detection distance of an agent."""

def get_agent_max_neighbors(agent_id: int) -> int:
    """Get the maximum number of neighbors considered by an agent."""

def get_agent_time_horizon(agent_id: int) -> float:
    """Get the time horizon for agent-agent collision avoidance."""

def get_agent_time_horizon_obst(agent_id: int) -> float:
    """Get the time horizon for agent-obstacle collision avoidance."""
```

##### Agent Parameter Setters

```python
def set_agent_position(agent_id: int, position: Vector2) -> None:
    """Set the position of an agent."""

def set_agent_velocity(agent_id: int, velocity: Vector2) -> None:
    """Set the velocity of an agent."""

def set_agent_pref_velocity(agent_id: int, pref_velocity: Vector2) -> None:
    """Set the preferred velocity of an agent.
    
    The preferred velocity is what the agent would like to do
    if there were no other agents or obstacles around.
    """

def set_agent_radius(agent_id: int, radius: float) -> None:
    """Set the radius of an agent."""

def set_agent_max_speed(agent_id: int, max_speed: float) -> None:
    """Set the maximum speed of an agent."""

def set_agent_neighbor_dist(agent_id: int, neighbor_dist: float) -> None:
    """Set the neighbor detection distance of an agent."""

def set_agent_max_neighbors(agent_id: int, max_neighbors: int) -> None:
    """Set the maximum number of neighbors for an agent."""

def set_agent_time_horizon(agent_id: int, time_horizon: float) -> None:
    """Set the time horizon for agent-agent collision avoidance."""

def set_agent_time_horizon_obst(agent_id: int, time_horizon_obst: float) -> None:
    """Set the time horizon for agent-obstacle collision avoidance."""

def set_agent_defaults(neighbor_dist: float, max_neighbors: int,
                      time_horizon: float, time_horizon_obst: float,
                      radius: float, max_speed: float,
                      velocity: Vector2 = Vector2()) -> None:
    """Set default parameters for subsequently added agents."""
```

#### Neighbor Information

```python
def get_agent_num_agent_neighbors(agent_id: int) -> int:
    """Get the number of agent neighbors of an agent."""

def get_agent_num_obstacle_neighbors(agent_id: int) -> int:
    """Get the number of obstacle neighbors of an agent."""

def get_agent_agent_neighbor(agent_id: int, neighbor_index: int) -> int:
    """Get the ID of a specific agent neighbor.
    
    Args:
        agent_id: The agent whose neighbor to query
        neighbor_index: Index of the neighbor (0 to num_agent_neighbors - 1)
        
    Returns:
        Agent ID of the neighbor
    """

def get_agent_obstacle_neighbor(agent_id: int, neighbor_index: int) -> int:
    """Get the ID of a specific obstacle neighbor.
    
    Args:
        agent_id: The agent whose neighbor to query
        neighbor_index: Index of the neighbor (0 to num_obstacle_neighbors - 1)
        
    Returns:
        Obstacle vertex ID
    """
```

#### ORCA Constraints

```python
def get_agent_orca_line(agent_id: int, line_index: int) -> Line:
    """Get a specific ORCA constraint line for an agent.
    
    ORCA lines represent the velocity constraints that ensure
    collision avoidance with other agents and obstacles.
    
    Args:
        agent_id: The agent whose ORCA line to query
        line_index: Index of the ORCA line
        
    Returns:
        Line object with point and direction attributes
    """
```

#### Obstacle Management

```python
def add_obstacle(vertices: List[Vector2]) -> None:
    """Add a polygonal obstacle to the simulation.
    
    Args:
        vertices: List of vertices in counterclockwise order
        
    Note:
        Obstacles must be added before calling process_obstacles().
        The polygon should be simple (non-self-intersecting).
    """

def process_obstacles() -> None:
    """Process all added obstacles for collision avoidance.
    
    This must be called after adding all obstacles and before
    starting the simulation. No obstacles can be added after
    calling this method.
    """

def get_num_obstacle_vertices() -> int:
    """Get the total number of obstacle vertices."""

def get_obstacle_vertex(vertex_id: int) -> Vector2:
    """Get the position of a specific obstacle vertex."""

def get_next_obstacle_vertex_no(vertex_id: int) -> int:
    """Get the ID of the next vertex in the same obstacle."""

def get_prev_obstacle_vertex_no(vertex_id: int) -> int:
    """Get the ID of the previous vertex in the same obstacle."""
```

#### Simulation Control

```python
def do_step() -> None:
    """Advance the simulation by one time step.
    
    This computes new velocities for all agents based on their
    preferred velocities and ORCA constraints, then updates
    agent positions.
    """

def get_global_time() -> float:
    """Get the current simulation time."""

def get_time_step() -> float:
    """Get the simulation time step."""

def set_time_step(time_step: float) -> None:
    """Set the simulation time step."""
```

#### Visibility Queries

```python
def query_visibility(point1: Vector2, point2: Vector2, radius: float = 0.0) -> float:
    """Check visibility between two points.
    
    Args:
        point1: First point
        point2: Second point
        radius: Radius for collision checking (default: 0.0)
        
    Returns:
        Distance along line from point1 toward point2 before hitting obstacle.
        Returns full distance if no obstacles block the path.
    """
```

### Vector2

2D vector class for positions, velocities, and directions.

```python
from rvo2_rl.rvo2 import Vector2

# Create vectors
v1 = Vector2()           # (0, 0)
v2 = Vector2(3.0, 4.0)   # (3, 4)

# Access components
x = v2.x()
y = v2.y()

# Vector operations
v3 = v1 + v2             # Addition
v4 = v2 - v1             # Subtraction
v5 = v2 * 2.0            # Scalar multiplication
v6 = 2.0 * v2            # Scalar multiplication (reversed)

# String representation
print(v2)                # (3.0, 4.0)
```

#### Vector Operations

| Operation | Syntax | Description |
|-----------|--------|-------------|
| Construction | `Vector2(x, y)` | Create vector with components |
| Addition | `v1 + v2` | Component-wise addition |
| Subtraction | `v1 - v2` | Component-wise subtraction |
| Scalar multiplication | `v * scalar` or `scalar * v` | Scale vector |
| Component access | `v.x()`, `v.y()` | Get x or y component |
| String representation | `str(v)` | Get string like "(x, y)" |

### Line

Represents a line in 2D space, used for ORCA constraints.

```python
from rvo2_rl.rvo2 import Line, Vector2

# Line objects are typically returned by get_agent_orca_line()
line = sim.get_agent_orca_line(agent_id, line_index)

# Access line properties
point = line.point          # Point on the line
direction = line.direction  # Direction vector of the line
```

## Usage Patterns

### Basic Simulation Loop

```python
import rvo2_rl
from rvo2_rl.rvo2 import RVOSimulator, Vector2

# Setup simulation
sim = RVOSimulator()
sim.set_time_step(0.25)

# Set default parameters for agents
sim.set_agent_defaults(
    neighbor_dist=15.0,
    max_neighbors=10,
    time_horizon=5.0,
    time_horizon_obst=5.0,
    radius=0.5,
    max_speed=2.0
)

# Add agents
agent_positions = [Vector2(-10, 0), Vector2(10, 0)]
goals = [Vector2(10, 0), Vector2(-10, 0)]

for pos in agent_positions:
    sim.add_agent(pos)

# Add obstacles (optional)
obstacle_vertices = [
    Vector2(-1, -5), Vector2(1, -5), 
    Vector2(1, 5), Vector2(-1, 5)
]
sim.add_obstacle(obstacle_vertices)
sim.process_obstacles()

# Simulation loop
for step in range(100):
    # Set preferred velocities (goal-seeking behavior)
    for i in range(sim.get_num_agents()):
        goal = goals[i]
        current_pos = sim.get_agent_position(i)
        
        # Simple goal-seeking preferred velocity
        direction = Vector2(goal.x() - current_pos.x(), 
                           goal.y() - current_pos.y())
        
        # Normalize and set as preferred velocity
        distance = (direction.x()**2 + direction.y()**2)**0.5
        if distance > 0.1:  # Not at goal yet
            pref_vel = Vector2(direction.x() / distance, 
                              direction.y() / distance)
            sim.set_agent_pref_velocity(i, pref_vel)
        else:
            sim.set_agent_pref_velocity(i, Vector2(0, 0))
    
    # Step simulation
    sim.do_step()
    
    # Print positions
    print(f"Step {step}:")
    for i in range(sim.get_num_agents()):
        pos = sim.get_agent_position(i)
        vel = sim.get_agent_velocity(i)
        print(f"  Agent {i}: pos=({pos.x():.2f}, {pos.y():.2f}), "
              f"vel=({vel.x():.2f}, {vel.y():.2f})")
```

### Dynamic Agent Management

```python
# Start with empty simulation
sim = RVOSimulator()
sim.set_time_step(0.1)

# Add agents dynamically during simulation
for step in range(200):
    # Add new agent every 10 steps
    if step % 10 == 0 and step < 100:
        new_pos = Vector2(step * 0.1, 0)
        agent_id = sim.add_agent(new_pos, 
                                neighbor_dist=5.0,
                                max_neighbors=3,
                                time_horizon=2.0,
                                time_horizon_obst=2.0,
                                radius=0.3,
                                max_speed=1.5)
        print(f"Added agent {agent_id} at step {step}")
    
    # Set preferred velocities for all agents
    for i in range(sim.get_num_agents()):
        # Move toward positive x direction
        sim.set_agent_pref_velocity(i, Vector2(1.0, 0.0))
    
    # Step simulation
    sim.do_step()
```

### Analyzing ORCA Constraints

```python
def analyze_agent_constraints(sim, agent_id):
    """Analyze the ORCA constraints for a specific agent."""
    
    num_agent_neighbors = sim.get_agent_num_agent_neighbors(agent_id)
    num_obstacle_neighbors = sim.get_agent_num_obstacle_neighbors(agent_id)
    
    print(f"Agent {agent_id} constraints:")
    print(f"  Agent neighbors: {num_agent_neighbors}")
    print(f"  Obstacle neighbors: {num_obstacle_neighbors}")
    
    # Examine agent neighbors
    for i in range(num_agent_neighbors):
        neighbor_id = sim.get_agent_agent_neighbor(agent_id, i)
        neighbor_pos = sim.get_agent_position(neighbor_id)
        print(f"    Neighbor {neighbor_id} at ({neighbor_pos.x():.2f}, {neighbor_pos.y():.2f})")
    
    # Examine ORCA lines
    total_lines = num_agent_neighbors + num_obstacle_neighbors
    for i in range(total_lines):
        line = sim.get_agent_orca_line(agent_id, i)
        print(f"    ORCA line {i}: point=({line.point.x():.2f}, {line.point.y():.2f}), "
              f"direction=({line.direction.x():.2f}, {line.direction.y():.2f})")

# Use during simulation
for step in range(10):
    # ... set preferred velocities ...
    sim.do_step()
    
    if step == 5:  # Analyze at step 5
        analyze_agent_constraints(sim, 0)
```

### Complex Obstacle Scenarios

```python
def create_maze_environment(sim):
    """Create a maze-like environment with multiple obstacles."""
    
    # Outer walls
    walls = [
        # Bottom wall
        [Vector2(-20, -20), Vector2(20, -20), Vector2(20, -18), Vector2(-20, -18)],
        # Top wall  
        [Vector2(-20, 18), Vector2(20, 18), Vector2(20, 20), Vector2(-20, 20)],
        # Left wall
        [Vector2(-20, -18), Vector2(-18, -18), Vector2(-18, 18), Vector2(-20, 18)],
        # Right wall
        [Vector2(18, -18), Vector2(20, -18), Vector2(20, 18), Vector2(18, 18)]
    ]
    
    # Internal obstacles
    internal_obstacles = [
        # Central pillar
        [Vector2(-2, -2), Vector2(2, -2), Vector2(2, 2), Vector2(-2, 2)],
        # Side barriers
        [Vector2(-10, -10), Vector2(-5, -10), Vector2(-5, -5), Vector2(-10, -5)],
        [Vector2(5, 5), Vector2(10, 5), Vector2(10, 10), Vector2(5, 10)]
    ]
    
    # Add all obstacles
    for wall in walls + internal_obstacles:
        sim.add_obstacle(wall)
    
    sim.process_obstacles()

# Create maze simulation
sim = RVOSimulator()
sim.set_time_step(0.1)
create_maze_environment(sim)

# Add agents at different starting points
start_positions = [
    Vector2(-15, -15), Vector2(15, 15),
    Vector2(-15, 15), Vector2(15, -15)
]

goals = [
    Vector2(15, 15), Vector2(-15, -15),
    Vector2(15, -15), Vector2(-15, 15)
]

for pos in start_positions:
    sim.add_agent(pos, neighbor_dist=8.0, max_neighbors=5, 
                  time_horizon=3.0, time_horizon_obst=2.0,
                  radius=0.5, max_speed=1.5)

# Simulation with intelligent pathfinding
for step in range(500):
    for i in range(sim.get_num_agents()):
        current_pos = sim.get_agent_position(i)
        goal = goals[i]
        
        # Simple potential field navigation
        goal_force = Vector2(goal.x() - current_pos.x(), 
                           goal.y() - current_pos.y())
        
        # Normalize goal force
        distance = (goal_force.x()**2 + goal_force.y()**2)**0.5
        if distance > 0.1:
            goal_force = Vector2(goal_force.x() / distance,
                               goal_force.y() / distance)
            sim.set_agent_pref_velocity(i, goal_force)
        else:
            sim.set_agent_pref_velocity(i, Vector2(0, 0))
    
    sim.do_step()
    
    # Check if all agents reached goals
    all_arrived = True
    for i in range(sim.get_num_agents()):
        pos = sim.get_agent_position(i)
        goal = goals[i]
        dist = ((pos.x() - goal.x())**2 + (pos.y() - goal.y())**2)**0.5
        if dist > 1.0:
            all_arrived = False
            break
    
    if all_arrived:
        print(f"All agents reached their goals at step {step}")
        break
```

## Parameter Guidelines

### Agent Parameters

| Parameter | Typical Range | Effect |
|-----------|---------------|--------|
| `neighborDist` | 5.0 - 20.0 | Larger values: more neighbors considered, smoother but slower |
| `maxNeighbors` | 3 - 15 | More neighbors: better avoidance but higher computation |
| `timeHorizon` | 1.0 - 10.0 | Larger values: earlier reaction, more conservative |
| `timeHorizonObst` | 0.5 - 5.0 | Larger values: earlier obstacle avoidance |
| `radius` | 0.1 - 2.0 | Agent size for collision detection |
| `maxSpeed` | 0.5 - 5.0 | Maximum achievable speed |

### Global Parameters

| Parameter | Typical Range | Effect |
|-----------|---------------|--------|
| `timeStep` | 0.05 - 0.5 | Smaller values: more accurate but slower simulation |

## Integration with RL Extensions

The RVO2 core is automatically used by the RL extensions. You can access it directly:

```python
from rvo2_rl.rl import RVO2RLWrapper

wrapper = RVO2RLWrapper()
# ... setup agents and goals ...
wrapper.initialize()

# Access underlying RVO2 simulator
sim = wrapper.get_simulator()

# Use core API for advanced control
for i in range(sim.get_num_agents()):
    # Get detailed neighbor information
    num_neighbors = sim.get_agent_num_agent_neighbors(i)
    print(f"Agent {i} has {num_neighbors} neighbors")
    
    # Access ORCA constraints
    for j in range(num_neighbors):
        line = sim.get_agent_orca_line(i, j)
        # Analyze constraint...
```

This direct access allows you to implement custom behaviors, analyze collision avoidance constraints, or debug simulation issues while still benefiting from the RL-optimized observation interfaces.
