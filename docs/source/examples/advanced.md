# Advanced Examples

This section covers advanced features including LiDAR integration, complex obstacle navigation, and performance optimization techniques.

## Example 1: LiDAR-Based Navigation

Demonstrate LiDAR sensing for autonomous navigation in complex environments.

```python
import numpy as np
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2

class LidarNavigationController:
    """Advanced controller using LiDAR for navigation."""
    
    def __init__(self, lidar_range=10.0, safety_distance=1.5):
        self.lidar_range = lidar_range
        self.safety_distance = safety_distance
        self.max_speed = 1.5
    
    def analyze_lidar(self, lidar_data):
        """Analyze LiDAR data to find safe directions."""
        safe_directions = []
        obstacle_distances = []
        
        for i, row in enumerate(lidar_data):
            if len(row) == 3:  # With mask
                angle, distance, mask = row
                if mask < 0.5:  # Invalid reading
                    continue
            else:  # Without mask
                angle, distance = row
            
            # Convert normalized distance back to real distance
            real_distance = distance * self.lidar_range
            obstacle_distances.append((angle, real_distance))
            
            # Check if direction is safe
            if real_distance > self.safety_distance:
                safe_directions.append(angle)
        
        return safe_directions, obstacle_distances
    
    def find_best_direction(self, lidar_data, goal_direction):
        """Find the best direction considering both goal and obstacles."""
        safe_directions, obstacles = self.analyze_lidar(lidar_data)
        
        if not safe_directions:
            # No safe directions - emergency stop
            return 0.0, 0.0
        
        # Find the safe direction closest to the goal direction
        best_angle = goal_direction
        min_angle_diff = float('inf')
        
        for angle in safe_directions:
            angle_diff = abs(self.angle_difference(angle, goal_direction))
            if angle_diff < min_angle_diff:
                min_angle_diff = angle_diff
                best_angle = angle
        
        # Calculate velocity in best direction
        speed = self.max_speed
        
        # Reduce speed if obstacles are close in chosen direction
        for angle, distance in obstacles:
            if abs(self.angle_difference(angle, best_angle)) < 0.5:  # Within 30 degrees
                if distance < self.safety_distance * 2:
                    speed *= (distance / (self.safety_distance * 2))
        
        vx = speed * np.cos(best_angle)
        vy = speed * np.sin(best_angle)
        
        return vx, vy
    
    def angle_difference(self, angle1, angle2):
        """Calculate the smallest difference between two angles."""
        diff = angle1 - angle2
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff
    
    def calculate_goal_direction(self, current_pos, goal_pos):
        """Calculate direction toward goal."""
        dx = goal_pos.x() - current_pos.x()
        dy = goal_pos.y() - current_pos.y()
        return np.arctan2(dy, dx)

def create_complex_environment(wrapper):
    """Create a complex environment with multiple obstacles."""
    sim = wrapper.get_simulator()
    
    # Create a maze-like environment
    obstacles = [
        # Outer walls
        [Vector2(-15, -15), Vector2(15, -15), Vector2(15, -13), Vector2(-15, -13)],  # Bottom
        [Vector2(-15, 13), Vector2(15, 13), Vector2(15, 15), Vector2(-15, 15)],      # Top
        [Vector2(-15, -13), Vector2(-13, -13), Vector2(-13, 13), Vector2(-15, 13)], # Left
        [Vector2(13, -13), Vector2(15, -13), Vector2(15, 13), Vector2(13, 13)],     # Right
        
        # Internal obstacles
        [Vector2(-8, -8), Vector2(-3, -8), Vector2(-3, -3), Vector2(-8, -3)],       # Large block
        [Vector2(3, 3), Vector2(8, 3), Vector2(8, 8), Vector2(3, 8)],              # Large block
        [Vector2(-2, -2), Vector2(2, -2), Vector2(2, 2), Vector2(-2, 2)],          # Central block
        
        # Narrow passages
        [Vector2(-1, 5), Vector2(1, 5), Vector2(1, 10), Vector2(-1, 10)],          # Vertical barrier
        [Vector2(-10, -1), Vector2(-5, -1), Vector2(-5, 1), Vector2(-10, 1)],      # Horizontal barrier
        
        # Scattered obstacles
        [Vector2(5, -5), Vector2(7, -5), Vector2(7, -3), Vector2(5, -3)],
        [Vector2(-7, 5), Vector2(-5, 5), Vector2(-5, 7), Vector2(-7, 7)],
    ]
    
    for obstacle in obstacles:
        sim.add_obstacle(obstacle)
    
    sim.process_obstacles()
    print("Complex environment created with multiple obstacles")

def lidar_navigation_demo():
    """Demonstrate LiDAR-based navigation."""
    
    # Create wrapper with high-resolution LiDAR
    wrapper = RVO2RLWrapper(
        time_step=0.1,
        neighbor_dist=12.0,
        max_neighbors=5,
        radius=0.4,
        max_speed=1.5,
        mode=ObsMode.Cartesian,
        use_lidar=True,
        lidar_count=72,      # 5-degree resolution
        lidar_range=8.0,
        use_obs_mask=True
    )
    
    # Create complex environment
    create_complex_environment(wrapper)
    
    # Add agent at starting position
    start_pos = Vector2(-12, -12)
    goal_pos = Vector2(12, 12)
    agent_id = wrapper.add_agent(start_pos)
    
    wrapper.set_goals([goal_pos])
    wrapper.initialize()
    
    # Create LiDAR controller
    controller = LidarNavigationController(lidar_range=8.0, safety_distance=1.0)
    
    print("LiDAR Navigation Demonstration")
    print(f"Start: ({start_pos.x()}, {start_pos.y()})")
    print(f"Goal:  ({goal_pos.x()}, {goal_pos.y()})")
    print("="*50)
    
    path_points = []
    
    for step in range(500):
        # Get current position
        current_pos = wrapper.get_simulator().get_agent_position(agent_id)
        path_points.append((current_pos.x(), current_pos.y()))
        
        # Get LiDAR data
        lidar_data = wrapper.get_lidar(agent_id)
        
        # Calculate goal direction
        goal_direction = controller.calculate_goal_direction(current_pos, goal_pos)
        
        # Find best direction using LiDAR
        vx, vy = controller.find_best_direction(lidar_data, goal_direction)
        
        # Apply velocity
        wrapper.set_preferred_velocity(agent_id, Vector2(vx, vy))
        
        # Step simulation
        wrapper.get_simulator().do_step()
        
        # Check progress
        distance_to_goal = wrapper.get_distance_to_goal(agent_id)
        
        if step % 50 == 0:
            print(f"Step {step}: pos=({current_pos.x():.2f}, {current_pos.y():.2f}), "
                  f"dist_to_goal={distance_to_goal:.2f}")
            
            # Analyze LiDAR data
            safe_dirs, obstacles = controller.analyze_lidar(lidar_data)
            closest_obstacle = min([dist for _, dist in obstacles]) if obstacles else float('inf')
            print(f"  LiDAR: {len(safe_dirs)} safe directions, closest obstacle: {closest_obstacle:.2f}")
        
        # Check if reached goal
        if distance_to_goal < 1.0:
            print(f"\\nGoal reached at step {step}!")
            break
        
        # Check if stuck (not moving)
        if step > 100 and len(path_points) >= 10:
            recent_positions = path_points[-10:]
            movement = sum(np.sqrt((recent_positions[i][0] - recent_positions[i-1][0])**2 + 
                                 (recent_positions[i][1] - recent_positions[i-1][1])**2) 
                          for i in range(1, len(recent_positions)))
            
            if movement < 0.5:  # Very little movement
                print(f"\\nAgent appears stuck at step {step}")
                break
    
    # Analyze path
    total_distance = sum(np.sqrt((path_points[i][0] - path_points[i-1][0])**2 + 
                               (path_points[i][1] - path_points[i-1][1])**2) 
                        for i in range(1, len(path_points)))
    
    straight_line_distance = np.sqrt((goal_pos.x() - start_pos.x())**2 + 
                                   (goal_pos.y() - start_pos.y())**2)
    
    print(f"\\nPath Analysis:")
    print(f"  Total path length: {total_distance:.2f}")
    print(f"  Straight-line distance: {straight_line_distance:.2f}")
    print(f"  Path efficiency: {straight_line_distance/total_distance*100:.1f}%")

# Run the demo
lidar_navigation_demo()
```

## Example 2: Dynamic Obstacle Avoidance

Agents navigate around moving obstacles and other dynamic elements.

```python
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2
import numpy as np

class DynamicObstacle:
    """Represents a moving obstacle."""
    
    def __init__(self, initial_pos, size=1.0, speed=0.5):
        self.position = initial_pos
        self.size = size
        self.speed = speed
        self.direction = np.random.uniform(0, 2*np.pi)
        self.vertices = self._create_vertices()
    
    def _create_vertices(self):
        """Create vertices for a square obstacle."""
        half_size = self.size / 2
        return [
            Vector2(self.position.x() - half_size, self.position.y() - half_size),
            Vector2(self.position.x() + half_size, self.position.y() - half_size),
            Vector2(self.position.x() + half_size, self.position.y() + half_size),
            Vector2(self.position.x() - half_size, self.position.y() + half_size)
        ]
    
    def update(self, bounds=(-10, 10, -10, 10)):
        """Update obstacle position."""
        # Move in current direction
        dx = self.speed * np.cos(self.direction)
        dy = self.speed * np.sin(self.direction)
        
        new_x = self.position.x() + dx
        new_y = self.position.y() + dy
        
        # Bounce off boundaries
        if new_x < bounds[0] or new_x > bounds[1]:
            self.direction = np.pi - self.direction
            new_x = max(bounds[0], min(bounds[1], new_x))
        
        if new_y < bounds[2] or new_y > bounds[3]:
            self.direction = -self.direction
            new_y = max(bounds[2], min(bounds[3], new_y))
        
        self.position = Vector2(new_x, new_y)
        self.vertices = self._create_vertices()

class AdaptiveAgent:
    """Agent that adapts its behavior based on environment dynamics."""
    
    def __init__(self, agent_id, wrapper):
        self.agent_id = agent_id
        self.wrapper = wrapper
        self.stuck_counter = 0
        self.last_position = None
        self.exploration_mode = False
        self.exploration_direction = 0
    
    def update_behavior(self, step):
        """Update agent behavior based on current situation."""
        current_pos = self.wrapper.get_simulator().get_agent_position(self.agent_id)
        
        # Check if stuck
        if self.last_position is not None:
            movement = np.sqrt((current_pos.x() - self.last_position.x())**2 + 
                             (current_pos.y() - self.last_position.y())**2)
            
            if movement < 0.05:  # Very small movement
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0
                self.exploration_mode = False
        
        # Enter exploration mode if stuck
        if self.stuck_counter > 20:
            self.exploration_mode = True
            if self.stuck_counter == 21:  # First time entering exploration
                self.exploration_direction = np.random.uniform(0, 2*np.pi)
        
        if self.exploration_mode:
            # Exploration behavior: move in a different direction
            vx = 1.0 * np.cos(self.exploration_direction)
            vy = 1.0 * np.sin(self.exploration_direction)
            self.wrapper.set_preferred_velocity(self.agent_id, Vector2(vx, vy))
            
            # Change exploration direction occasionally
            if step % 30 == 0:
                self.exploration_direction += np.random.uniform(-np.pi/4, np.pi/4)
        else:
            # Normal goal-seeking behavior with dynamic obstacle awareness
            goal_pos = self.wrapper.get_goal(self.agent_id)
            goal_vector = Vector2(goal_pos[0], goal_pos[1])
            
            # Get neighbors and check for dangerous situations
            neighbors = self.wrapper.get_neighbors(self.agent_id)
            danger_level = self._assess_danger(neighbors)
            
            dx = goal_vector.x() - current_pos.x()
            dy = goal_vector.y() - current_pos.y()
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance > 0.1:
                # Adjust speed based on danger level
                base_speed = 1.2
                adjusted_speed = base_speed * (1.0 - danger_level * 0.5)
                
                # Add some randomness when in danger
                if danger_level > 0.5:
                    noise_x = np.random.uniform(-0.3, 0.3)
                    noise_y = np.random.uniform(-0.3, 0.3)
                    dx += noise_x
                    dy += noise_y
                    distance = np.sqrt(dx**2 + dy**2)
                
                vx = (dx / distance) * adjusted_speed
                vy = (dy / distance) * adjusted_speed
                self.wrapper.set_preferred_velocity(self.agent_id, Vector2(vx, vy))
            else:
                self.wrapper.set_preferred_velocity(self.agent_id, Vector2(0, 0))
        
        self.last_position = current_pos
    
    def _assess_danger(self, neighbors):
        """Assess danger level based on neighbor proximity and velocities."""
        danger = 0.0
        
        # Check if masks are enabled by examining data structure
        has_mask = neighbors.shape[1] == 7  # 7 columns means mask is included
        
        for i in range(neighbors.shape[0]):
            if has_mask and neighbors[i][-1] < 0.5:
                continue  # Invalid neighbor
            
            pos_x, pos_y = neighbors[i][0], neighbors[i][1]
            distance = np.sqrt(pos_x**2 + pos_y**2)
            
            if distance < 2.0:  # Close neighbor
                proximity_danger = (2.0 - distance) / 2.0
                
                # Check if neighbor is approaching
                vel_x, vel_y = neighbors[i][2], neighbors[i][3]
                relative_vel = pos_x * vel_x + pos_y * vel_y  # Dot product
                
                if relative_vel < 0:  # Approaching
                    proximity_danger *= 1.5
                
                danger = max(danger, proximity_danger)
        
        return min(1.0, danger)

def dynamic_obstacle_demo():
    """Demonstrate navigation with dynamic obstacles."""
    
    wrapper = RVO2RLWrapper(
        time_step=0.1,
        neighbor_dist=6.0,
        max_neighbors=8,
        radius=0.3,
        max_speed=1.5,
        mode=ObsMode.Cartesian,
        use_obs_mask=True
    )
    
    # Add static obstacles
    sim = wrapper.get_simulator()
    static_obstacles = [
        [Vector2(-5, -1), Vector2(-3, -1), Vector2(-3, 1), Vector2(-5, 1)],
        [Vector2(3, -1), Vector2(5, -1), Vector2(5, 1), Vector2(3, 1)]
    ]
    
    for obstacle in static_obstacles:
        sim.add_obstacle(obstacle)
    
    sim.process_obstacles()
    
    # Create dynamic obstacles (simulated as additional agents)
    dynamic_obstacles = [
        DynamicObstacle(Vector2(-2, 3), size=1.2, speed=0.3),
        DynamicObstacle(Vector2(2, -3), size=1.0, speed=0.4),
        DynamicObstacle(Vector2(0, 0), size=0.8, speed=0.2)
    ]
    
    # Add regular agents
    num_agents = 6
    agent_ids = []
    adaptive_agents = []
    
    # Start positions
    start_positions = [
        Vector2(-8, -8), Vector2(-8, 8), Vector2(8, -8),
        Vector2(8, 8), Vector2(-8, 0), Vector2(8, 0)
    ]
    
    # Goal positions (opposite corners/sides)
    goal_positions = [
        Vector2(8, 8), Vector2(8, -8), Vector2(-8, 8),
        Vector2(-8, -8), Vector2(8, 0), Vector2(-8, 0)
    ]
    
    for i in range(num_agents):
        agent_id = wrapper.add_agent(start_positions[i])
        agent_ids.append(agent_id)
        adaptive_agents.append(AdaptiveAgent(agent_id, wrapper))
    
    # Add dynamic obstacles as special "agents"
    obstacle_agent_ids = []
    for obstacle in dynamic_obstacles:
        obs_id = wrapper.add_agent(obstacle.position, 
                                 neighbor_dist=1.0,  # Obstacles don't need to see far
                                 max_neighbors=2,
                                 time_horizon=1.0,
                                 time_horizon_obst=1.0,
                                 radius=obstacle.size/2,
                                 max_speed=obstacle.speed * 2)
        obstacle_agent_ids.append(obs_id)
    
    # Set goals
    all_goals = goal_positions + [Vector2(0, 0) for _ in dynamic_obstacles]
    wrapper.set_goals(all_goals)
    wrapper.initialize()
    
    print("Dynamic Obstacle Avoidance Demonstration")
    print(f"Regular agents: {num_agents}")
    print(f"Dynamic obstacles: {len(dynamic_obstacles)}")
    print("="*50)
    
    for step in range(400):
        # Update dynamic obstacles
        for i, obstacle in enumerate(dynamic_obstacles):
            obstacle.update(bounds=(-9, 9, -9, 9))
            
            # Update obstacle agent position
            obs_agent_id = obstacle_agent_ids[i]
            wrapper.get_simulator().set_agent_position(obs_agent_id, obstacle.position)
            
            # Set obstacle movement as preferred velocity
            dx = obstacle.speed * np.cos(obstacle.direction)
            dy = obstacle.speed * np.sin(obstacle.direction)
            wrapper.set_preferred_velocity(obs_agent_id, Vector2(dx, dy))
        
        # Update regular agents with adaptive behavior
        for adaptive_agent in adaptive_agents:
            adaptive_agent.update_behavior(step)
        
        # Step simulation
        wrapper.get_simulator().do_step()
        
        # Progress reporting
        if step % 50 == 0:
            print(f"\\nStep {step}:")
            
            for i, agent_id in enumerate(agent_ids):
                distance = wrapper.get_distance_to_goal(agent_id)
                exploration_mode = adaptive_agents[i].exploration_mode
                stuck_count = adaptive_agents[i].stuck_counter
                
                mode_str = "exploring" if exploration_mode else "goal-seeking"
                print(f"  Agent {i}: dist={distance:.2f}, mode={mode_str}, stuck={stuck_count}")
            
            # Dynamic obstacle positions
            print("  Dynamic obstacles:")
            for i, obstacle in enumerate(dynamic_obstacles):
                print(f"    Obstacle {i}: ({obstacle.position.x():.2f}, {obstacle.position.y():.2f})")
        
        # Check completion
        completed = sum(1 for agent_id in agent_ids 
                       if wrapper.get_distance_to_goal(agent_id) < 0.8)
        
        if completed == num_agents:
            print(f"\\nAll agents reached goals at step {step}!")
            break
    
    print("\\nDynamic obstacle demonstration completed!")

# Run the demo
dynamic_obstacle_demo()
```

## Example 3: Hierarchical Path Planning

Combine global path planning with local collision avoidance.

```python
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2
import numpy as np
from collections import deque

class AStarPlanner:
    """Simple A* path planner for global navigation."""
    
    def __init__(self, grid_size=50, cell_size=0.5):
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.grid = np.zeros((grid_size, grid_size))  # 0 = free, 1 = obstacle
    
    def set_obstacles(self, obstacle_list):
        """Mark obstacles in the grid."""
        self.grid.fill(0)  # Clear grid
        
        for obstacle in obstacle_list:
            # Simple obstacle marking (could be improved)
            for vertex in obstacle:
                grid_x = int((vertex.x() + self.grid_size * self.cell_size / 2) / self.cell_size)
                grid_y = int((vertex.y() + self.grid_size * self.cell_size / 2) / self.cell_size)
                
                # Mark obstacle and surrounding cells
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        x, y = grid_x + dx, grid_y + dy
                        if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                            self.grid[y, x] = 1
    
    def world_to_grid(self, world_pos):
        """Convert world coordinates to grid coordinates."""
        grid_x = int((world_pos.x() + self.grid_size * self.cell_size / 2) / self.cell_size)
        grid_y = int((world_pos.y() + self.grid_size * self.cell_size / 2) / self.cell_size)
        return max(0, min(self.grid_size-1, grid_x)), max(0, min(self.grid_size-1, grid_y))
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates."""
        world_x = (grid_x * self.cell_size) - (self.grid_size * self.cell_size / 2)
        world_y = (grid_y * self.cell_size) - (self.grid_size * self.cell_size / 2)
        return Vector2(world_x, world_y)
    
    def plan_path(self, start, goal):
        """Plan path using A* algorithm."""
        start_grid = self.world_to_grid(start)
        goal_grid = self.world_to_grid(goal)
        
        if self.grid[start_grid[1], start_grid[0]] == 1 or self.grid[goal_grid[1], goal_grid[0]] == 1:
            return []  # Start or goal in obstacle
        
        # A* implementation
        open_set = [(0, start_grid, [])]
        closed_set = set()
        
        while open_set:
            current_cost, current, path = open_set.pop(0)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            new_path = path + [current]
            
            if current == goal_grid:
                # Convert path to world coordinates
                world_path = [self.grid_to_world(x, y) for x, y in new_path]
                return world_path
            
            # Explore neighbors
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]:
                nx, ny = current[0] + dx, current[1] + dy
                
                if (0 <= nx < self.grid_size and 0 <= ny < self.grid_size and 
                    self.grid[ny, nx] == 0 and (nx, ny) not in closed_set):
                    
                    # Calculate cost
                    move_cost = 1.4 if dx != 0 and dy != 0 else 1.0  # Diagonal cost
                    g_cost = current_cost + move_cost
                    h_cost = abs(nx - goal_grid[0]) + abs(ny - goal_grid[1])  # Manhattan distance
                    f_cost = g_cost + h_cost
                    
                    open_set.append((f_cost, (nx, ny), new_path))
                    open_set.sort(key=lambda x: x[0])  # Sort by f_cost
        
        return []  # No path found

class HierarchicalAgent:
    """Agent using hierarchical planning: global path + local avoidance."""
    
    def __init__(self, agent_id, wrapper, planner):
        self.agent_id = agent_id
        self.wrapper = wrapper
        self.planner = planner
        self.global_path = []
        self.current_waypoint_index = 0
        self.waypoint_reached_distance = 1.0
        self.replan_counter = 0
    
    def set_goal(self, goal):
        """Set new goal and plan global path."""
        current_pos = self.wrapper.get_simulator().get_agent_position(self.agent_id)
        self.global_path = self.planner.plan_path(current_pos, goal)
        self.current_waypoint_index = 0
        
        if self.global_path:
            print(f"Agent {self.agent_id}: Planned path with {len(self.global_path)} waypoints")
        else:
            print(f"Agent {self.agent_id}: No path found!")
    
    def update_behavior(self, step):
        """Update agent behavior using hierarchical planning."""
        current_pos = self.wrapper.get_simulator().get_agent_position(self.agent_id)
        
        # Check if we need to replan (every 100 steps or if stuck)
        self.replan_counter += 1
        if self.replan_counter >= 100:
            self._replan_if_needed()
            self.replan_counter = 0
        
        if not self.global_path:
            # No path available, stop
            self.wrapper.set_preferred_velocity(self.agent_id, Vector2(0, 0))
            return
        
        # Get current target waypoint
        target_waypoint = self._get_current_waypoint()
        
        if target_waypoint is None:
            # Reached final goal
            self.wrapper.set_preferred_velocity(self.agent_id, Vector2(0, 0))
            return
        
        # Move toward current waypoint
        dx = target_waypoint.x() - current_pos.x()
        dy = target_waypoint.y() - current_pos.y()
        distance = np.sqrt(dx**2 + dy**2)
        
        if distance < self.waypoint_reached_distance:
            # Reached current waypoint, advance to next
            self.current_waypoint_index += 1
            target_waypoint = self._get_current_waypoint()
            
            if target_waypoint is not None:
                dx = target_waypoint.x() - current_pos.x()
                dy = target_waypoint.y() - current_pos.y()
                distance = np.sqrt(dx**2 + dy**2)
        
        # Set preferred velocity toward waypoint
        if distance > 0.1:
            speed = min(1.5, distance)  # Slow down when close
            vx = (dx / distance) * speed
            vy = (dy / distance) * speed
            self.wrapper.set_preferred_velocity(self.agent_id, Vector2(vx, vy))
        else:
            self.wrapper.set_preferred_velocity(self.agent_id, Vector2(0, 0))
    
    def _get_current_waypoint(self):
        """Get the current target waypoint."""
        if self.current_waypoint_index < len(self.global_path):
            return self.global_path[self.current_waypoint_index]
        return None
    
    def _replan_if_needed(self):
        """Replan path if necessary."""
        current_pos = self.wrapper.get_simulator().get_agent_position(self.agent_id)
        
        # Check if significantly off path
        if self.global_path and len(self.global_path) > self.current_waypoint_index:
            nearest_waypoint = self.global_path[self.current_waypoint_index]
            distance_to_path = np.sqrt((current_pos.x() - nearest_waypoint.x())**2 + 
                                     (current_pos.y() - nearest_waypoint.y())**2)
            
            if distance_to_path > 3.0:  # Too far from planned path
                # Replan from current position
                original_goal = self.global_path[-1] if self.global_path else None
                if original_goal:
                    print(f"Agent {self.agent_id}: Replanning due to deviation")
                    new_path = self.planner.plan_path(current_pos, original_goal)
                    if new_path:
                        self.global_path = new_path
                        self.current_waypoint_index = 0

def hierarchical_planning_demo():
    """Demonstrate hierarchical path planning."""
    
    wrapper = RVO2RLWrapper(
        time_step=0.15,
        neighbor_dist=8.0,
        max_neighbors=6,
        radius=0.4,
        max_speed=1.5,
        mode=ObsMode.Cartesian
    )
    
    # Create complex environment
    sim = wrapper.get_simulator()
    obstacles = [
        # Large central obstacle
        [Vector2(-3, -3), Vector2(3, -3), Vector2(3, 3), Vector2(-3, 3)],
        
        # Maze-like structure
        [Vector2(-8, -1), Vector2(-5, -1), Vector2(-5, 1), Vector2(-8, 1)],
        [Vector2(5, -1), Vector2(8, -1), Vector2(8, 1), Vector2(5, 1)],
        [Vector2(-1, 5), Vector2(1, 5), Vector2(1, 8), Vector2(-1, 8)],
        [Vector2(-1, -8), Vector2(1, -8), Vector2(1, -5), Vector2(-1, -5)],
        
        # Scattered obstacles
        [Vector2(-6, 4), Vector2(-4, 4), Vector2(-4, 6), Vector2(-6, 6)],
        [Vector2(4, -6), Vector2(6, -6), Vector2(6, -4), Vector2(4, -4)],
    ]
    
    for obstacle in obstacles:
        sim.add_obstacle(obstacle)
    
    sim.process_obstacles()
    
    # Create A* planner
    planner = AStarPlanner(grid_size=40, cell_size=0.5)
    planner.set_obstacles(obstacles)
    
    # Add agents
    num_agents = 4
    start_positions = [
        Vector2(-9, -9), Vector2(9, -9),
        Vector2(-9, 9), Vector2(9, 9)
    ]
    
    goal_positions = [
        Vector2(9, 9), Vector2(-9, 9),
        Vector2(9, -9), Vector2(-9, -9)
    ]
    
    agent_ids = []
    hierarchical_agents = []
    
    for i in range(num_agents):
        agent_id = wrapper.add_agent(start_positions[i])
        agent_ids.append(agent_id)
        
        hierarchical_agent = HierarchicalAgent(agent_id, wrapper, planner)
        hierarchical_agents.append(hierarchical_agent)
    
    # Set dummy goals for initialization
    wrapper.set_goals(goal_positions)
    wrapper.initialize()
    
    # Set actual goals for hierarchical planning
    for i, agent in enumerate(hierarchical_agents):
        agent.set_goal(goal_positions[i])
    
    print("Hierarchical Path Planning Demonstration")
    print(f"Agents: {num_agents}")
    print("="*50)
    
    for step in range(600):
        # Update all hierarchical agents
        for agent in hierarchical_agents:
            agent.update_behavior(step)
        
        # Step simulation
        wrapper.get_simulator().do_step()
        
        # Progress reporting
        if step % 80 == 0:
            print(f"\\nStep {step}:")
            
            for i, agent_id in enumerate(agent_ids):
                pos = wrapper.get_simulator().get_agent_position(agent_id)
                distance = wrapper.get_distance_to_goal(agent_id)
                
                # Get planning status
                agent = hierarchical_agents[i]
                waypoint_info = f"waypoint {agent.current_waypoint_index}/{len(agent.global_path)}"
                
                print(f"  Agent {i}: pos=({pos.x():.2f}, {pos.y():.2f}), "
                      f"dist={distance:.2f}, {waypoint_info}")
        
        # Check completion
        completed = sum(1 for agent_id in agent_ids 
                       if wrapper.get_distance_to_goal(agent_id) < 1.5)
        
        if completed == num_agents:
            print(f"\\nAll agents reached goals at step {step}!")
            break
    
    print("\\nHierarchical planning demonstration completed!")

# Run the demo
hierarchical_planning_demo()
```

## Example 4: Performance Optimization for Large-Scale Simulations

Optimize PYRVO2-RL for simulations with hundreds of agents.

```python
import time
import numpy as np
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2
from rvo2_rl.util import openmp_max_threads, openmp_threads_used

class OptimizedSimulation:
    """Optimized simulation for large numbers of agents."""
    
    def __init__(self, num_agents=500):
        self.num_agents = num_agents
        
        # Optimized parameters for large simulations
        self.wrapper = RVO2RLWrapper(
            time_step=0.2,        # Larger time step for speed
            neighbor_dist=4.0,    # Reduced neighbor distance
            max_neighbors=4,      # Fewer neighbors for performance
            radius=0.25,          # Smaller agents
            max_speed=1.0,        # Moderate speed
            mode=ObsMode.Cartesian,
            use_obs_mask=False    # Disable masks for speed
        )
        
        self.agent_ids = []
        self.performance_metrics = {
            'setup_time': 0,
            'simulation_time': 0,
            'steps_per_second': 0,
            'agents_per_second': 0
        }
    
    def setup_scenario(self, scenario_type='crowd_evacuation'):
        """Set up different types of large-scale scenarios."""
        start_time = time.time()
        
        if scenario_type == 'crowd_evacuation':
            self._setup_evacuation_scenario()
        elif scenario_type == 'stadium_exit':
            self._setup_stadium_scenario()
        elif scenario_type == 'random_goals':
            self._setup_random_scenario()
        else:
            raise ValueError(f"Unknown scenario type: {scenario_type}")
        
        self.wrapper.initialize()
        self.performance_metrics['setup_time'] = time.time() - start_time
        
        print(f"Scenario setup completed in {self.performance_metrics['setup_time']:.2f}s")
        print(f"OpenMP threads: {openmp_threads_used()}/{openmp_max_threads()}")
    
    def _setup_evacuation_scenario(self):
        """Set up evacuation scenario with multiple exits."""
        # Distribute agents in a large rectangular area
        area_width, area_height = 40, 30
        agents_per_row = int(np.sqrt(self.num_agents * area_width / area_height))
        agents_per_col = self.num_agents // agents_per_row
        
        spacing_x = area_width / (agents_per_row + 1)
        spacing_y = area_height / (agents_per_col + 1)
        
        # Add some randomness to positions
        np.random.seed(42)
        
        for i in range(self.num_agents):
            row = i // agents_per_row
            col = i % agents_per_row
            
            base_x = -area_width/2 + (col + 1) * spacing_x
            base_y = -area_height/2 + (row + 1) * spacing_y
            
            # Add random offset
            x = base_x + np.random.uniform(-spacing_x*0.3, spacing_x*0.3)
            y = base_y + np.random.uniform(-spacing_y*0.3, spacing_y*0.3)
            
            agent_id = self.wrapper.add_agent(Vector2(x, y))
            self.agent_ids.append(agent_id)
        
        # Create exit goals
        exits = [
            Vector2(area_width/2 + 2, 0),      # Right exit
            Vector2(-area_width/2 - 2, 0),    # Left exit
            Vector2(0, area_height/2 + 2),    # Top exit
            Vector2(0, -area_height/2 - 2)    # Bottom exit
        ]
        
        # Assign agents to nearest exit
        goals = []
        for agent_id in self.agent_ids:
            pos = self.wrapper.get_simulator().get_agent_position(agent_id)
            
            nearest_exit = min(exits, key=lambda exit: 
                             (pos.x() - exit.x())**2 + (pos.y() - exit.y())**2)
            goals.append(nearest_exit)
        
        self.wrapper.set_goals(goals)
        print(f"Evacuation scenario: {self.num_agents} agents, {len(exits)} exits")
    
    def _setup_stadium_scenario(self):
        """Set up stadium-like scenario with circular arrangement."""
        # Agents arranged in concentric circles
        num_circles = int(np.sqrt(self.num_agents / 10))
        agents_per_circle = self.num_agents // num_circles
        
        np.random.seed(42)
        
        for circle in range(num_circles):
            radius = 5 + circle * 3
            
            for i in range(agents_per_circle):
                angle = 2 * np.pi * i / agents_per_circle
                angle += np.random.uniform(-0.1, 0.1)  # Small random offset
                
                x = radius * np.cos(angle)
                y = radius * np.sin(angle)
                
                agent_id = self.wrapper.add_agent(Vector2(x, y))
                self.agent_ids.append(agent_id)
        
        # All agents head to center
        center = Vector2(0, 0)
        goals = [center for _ in self.agent_ids]
        self.wrapper.set_goals(goals)
        
        print(f"Stadium scenario: {len(self.agent_ids)} agents in {num_circles} circles")
    
    def _setup_random_scenario(self):
        """Set up scenario with random positions and goals."""
        area_size = 50
        np.random.seed(42)
        
        # Random start positions
        for i in range(self.num_agents):
            x = np.random.uniform(-area_size/2, area_size/2)
            y = np.random.uniform(-area_size/2, area_size/2)
            
            agent_id = self.wrapper.add_agent(Vector2(x, y))
            self.agent_ids.append(agent_id)
        
        # Random goals
        goals = []
        for i in range(self.num_agents):
            x = np.random.uniform(-area_size/2, area_size/2)
            y = np.random.uniform(-area_size/2, area_size/2)
            goals.append(Vector2(x, y))
        
        self.wrapper.set_goals(goals)
        print(f"Random scenario: {self.num_agents} agents with random goals")
    
    def run_simulation(self, num_steps=200, progress_interval=50):
        """Run the optimized simulation."""
        print(f"\\nStarting simulation: {num_steps} steps, {self.num_agents} agents")
        print("="*60)
        
        start_time = time.time()
        
        for step in range(num_steps):
            # Use automatic preferred velocities for simplicity and speed
            self.wrapper.set_preferred_velocities()
            
            # Step simulation
            self.wrapper.get_simulator().do_step()
            
            # Progress reporting
            if step % progress_interval == 0:
                elapsed = time.time() - start_time
                if elapsed > 0:
                    steps_per_sec = (step + 1) / elapsed
                    agents_per_sec = steps_per_sec * self.num_agents
                    
                    print(f"Step {step:3d}: {steps_per_sec:6.1f} steps/s, "
                          f"{agents_per_sec:8.0f} agent-steps/s")
        
        total_time = time.time() - start_time
        
        # Calculate final metrics
        self.performance_metrics['simulation_time'] = total_time
        self.performance_metrics['steps_per_second'] = num_steps / total_time
        self.performance_metrics['agents_per_second'] = (num_steps * self.num_agents) / total_time
        
        self._print_performance_summary()
    
    def _print_performance_summary(self):
        """Print comprehensive performance summary."""
        print("\\n" + "="*60)
        print("PERFORMANCE SUMMARY")
        print("="*60)
        
        metrics = self.performance_metrics
        
        print(f"Setup time:           {metrics['setup_time']:.2f} seconds")
        print(f"Simulation time:      {metrics['simulation_time']:.2f} seconds")
        print(f"Steps per second:     {metrics['steps_per_second']:.1f}")
        print(f"Agent-steps per sec:  {metrics['agents_per_second']:,.0f}")
        
        # Performance categories
        if metrics['steps_per_second'] > 100:
            performance = "Excellent"
        elif metrics['steps_per_second'] > 50:
            performance = "Good"
        elif metrics['steps_per_second'] > 20:
            performance = "Acceptable"
        else:
            performance = "Poor"
        
        print(f"Performance rating:   {performance}")
        
        # System utilization
        print(f"\\nSystem Information:")
        print(f"OpenMP max threads:   {openmp_max_threads()}")
        print(f"OpenMP used threads:  {openmp_threads_used()}")
        
        if openmp_threads_used() < openmp_max_threads():
            print("⚠ Not using all available CPU threads")
        
        # Memory estimation
        bytes_per_agent = 400  # Rough estimate
        total_memory_mb = (self.num_agents * bytes_per_agent) / (1024 * 1024)
        print(f"Estimated memory use: {total_memory_mb:.1f} MB")

def performance_comparison():
    """Compare performance across different agent counts and configurations."""
    
    test_configs = [
        {'agents': 100, 'scenario': 'random_goals'},
        {'agents': 200, 'scenario': 'evacuation'},
        {'agents': 500, 'scenario': 'stadium_exit'},
    ]
    
    results = []
    
    for config in test_configs:
        print(f"\\n{'='*80}")
        print(f"PERFORMANCE TEST: {config['agents']} agents, {config['scenario']}")
        print(f"{'='*80}")
        
        sim = OptimizedSimulation(num_agents=config['agents'])
        sim.setup_scenario(config['scenario'])
        sim.run_simulation(num_steps=100, progress_interval=25)
        
        results.append({
            'agents': config['agents'],
            'scenario': config['scenario'],
            'steps_per_second': sim.performance_metrics['steps_per_second'],
            'agents_per_second': sim.performance_metrics['agents_per_second']
        })
    
    # Print comparison table
    print(f"\\n{'='*80}")
    print("PERFORMANCE COMPARISON")
    print(f"{'='*80}")
    print(f"{'Agents':<8} {'Scenario':<15} {'Steps/sec':<12} {'Agent-steps/sec':<15}")
    print("-" * 50)
    
    for result in results:
        print(f"{result['agents']:<8} {result['scenario']:<15} "
              f"{result['steps_per_second']:<12.1f} {result['agents_per_second']:<15.0f}")

# Run performance comparison
performance_comparison()
```

## Key Takeaways

### LiDAR Integration
- Use high-resolution LiDAR (72+ rays) for precise obstacle detection
- Implement safety margins and dynamic speed control
- Combine with goal-seeking for effective navigation

### Dynamic Environments
- Monitor agent movement to detect stuck situations
- Implement exploration modes for deadlock recovery
- Use adaptive behavior based on local conditions

### Hierarchical Planning
- Global path planning provides strategic guidance
- Local collision avoidance handles tactical movements
- Replanning ensures adaptability to changing conditions

### Performance Optimization
- Reduce `neighbor_dist` and `max_neighbors` for large simulations
- Use larger time steps when precision isn't critical
- Monitor OpenMP utilization for multi-threading efficiency

## Next Steps

- Study [Performance Guidelines](../technical/performance.md) for extreme-scale simulations
- Explore [Architecture Details](../technical/architecture.md) for understanding internal mechanisms
- Check [API References](../api/rl_extensions.md) for implementing custom advanced behaviors
