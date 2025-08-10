# Multi-Agent Scenarios

This section demonstrates complex multi-agent scenarios including formations, swarms, and large-scale simulations.

## Scenario 1: Agent Formation Control

Agents dynamically change between different formations.

```python
import numpy as np
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2

class FormationController:
    """Controller for managing agent formations."""
    
    def __init__(self, num_agents):
        self.num_agents = num_agents
        self.formations = {
            'line': self._line_formation,
            'circle': self._circle_formation,
            'diamond': self._diamond_formation,
            'grid': self._grid_formation
        }
    
    def _line_formation(self, center=(0, 0), spacing=1.5):
        """Create horizontal line formation."""
        positions = []
        start_x = center[0] - (self.num_agents - 1) * spacing / 2
        for i in range(self.num_agents):
            x = start_x + i * spacing
            y = center[1]
            positions.append(Vector2(x, y))
        return positions
    
    def _circle_formation(self, center=(0, 0), radius=3.0):
        """Create circular formation."""
        positions = []
        for i in range(self.num_agents):
            angle = 2 * np.pi * i / self.num_agents
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            positions.append(Vector2(x, y))
        return positions
    
    def _diamond_formation(self, center=(0, 0), size=3.0):
        """Create diamond formation."""
        positions = []
        if self.num_agents >= 4:
            # Four corners of diamond
            positions.extend([
                Vector2(center[0], center[1] + size),      # Top
                Vector2(center[0] + size, center[1]),      # Right
                Vector2(center[0], center[1] - size),      # Bottom
                Vector2(center[0] - size, center[1])       # Left
            ])
            
            # Fill remaining agents inside
            for i in range(4, self.num_agents):
                angle = 2 * np.pi * i / max(4, self.num_agents - 4)
                r = size * 0.5
                x = center[0] + r * np.cos(angle)
                y = center[1] + r * np.sin(angle)
                positions.append(Vector2(x, y))
        else:
            # Fallback to circle for few agents
            return self._circle_formation(center, size * 0.7)
        
        return positions[:self.num_agents]
    
    def _grid_formation(self, center=(0, 0), spacing=1.5):
        """Create grid formation."""
        positions = []
        grid_size = int(np.ceil(np.sqrt(self.num_agents)))
        
        start_x = center[0] - (grid_size - 1) * spacing / 2
        start_y = center[1] - (grid_size - 1) * spacing / 2
        
        for i in range(self.num_agents):
            row = i // grid_size
            col = i % grid_size
            x = start_x + col * spacing
            y = start_y + row * spacing
            positions.append(Vector2(x, y))
        
        return positions
    
    def get_formation(self, formation_name, **kwargs):
        """Get positions for a specific formation."""
        if formation_name in self.formations:
            return self.formations[formation_name](**kwargs)
        else:
            raise ValueError(f"Unknown formation: {formation_name}")

def formation_control_demo():
    """Demonstrate dynamic formation changes."""
    
    num_agents = 8
    wrapper = RVO2RLWrapper(
        time_step=0.2,
        neighbor_dist=10.0,
        max_neighbors=num_agents,  # All agents can see each other
        radius=0.3,
        max_speed=1.2,
        mode=ObsMode.Cartesian
    )
    
    controller = FormationController(num_agents)
    
    # Start in random positions
    agent_ids = []
    np.random.seed(42)
    for i in range(num_agents):
        x = np.random.uniform(-5, 5)
        y = np.random.uniform(-5, 5)
        agent_id = wrapper.add_agent(Vector2(x, y))
        agent_ids.append(agent_id)
    
    # Formation sequence
    formations = [
        ('line', {'center': (0, 0), 'spacing': 1.2}),
        ('circle', {'center': (5, 5), 'radius': 2.5}),
        ('diamond', {'center': (-3, 3), 'size': 2.0}),
        ('grid', {'center': (0, -4), 'spacing': 1.0})
    ]
    
    steps_per_formation = 60
    current_formation = 0
    
    print("Formation Control Demonstration")
    print("=" * 40)
    
    for step in range(steps_per_formation * len(formations)):
        # Check if time to change formation
        if step % steps_per_formation == 0:
            formation_name, formation_params = formations[current_formation]
            target_positions = controller.get_formation(formation_name, **formation_params)
            
            # Set new goals
            wrapper.set_goals(target_positions)
            
            print(f"\\nStep {step}: Changing to {formation_name} formation")
            print(f"Formation center: {formation_params.get('center', (0, 0))}")
            
            current_formation = (current_formation + 1) % len(formations)
        
        # Use automatic goal-seeking behavior
        wrapper.set_preferred_velocities()
        wrapper.get_simulator().do_step()
        
        # Print progress
        if step % 20 == 0:
            avg_distance = np.mean([wrapper.get_distance_to_goal(agent_id) 
                                   for agent_id in agent_ids])
            print(f"  Step {step}: Average distance to formation: {avg_distance:.2f}")
    
    print("\\nFormation control demonstration completed!")

# Run the demo
formation_control_demo()
```

## Scenario 2: Swarm Behavior with Leader-Follower

Implement flocking behavior with designated leaders.

```python
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2
import numpy as np

class SwarmController:
    """Controller implementing leader-follower swarm behavior."""
    
    def __init__(self, num_agents, num_leaders=2):
        self.num_agents = num_agents
        self.num_leaders = num_leaders
        self.leaders = list(range(num_leaders))  # First N agents are leaders
        self.followers = list(range(num_leaders, num_agents))
        
        # Swarm parameters
        self.separation_weight = 2.0
        self.alignment_weight = 1.0
        self.cohesion_weight = 1.0
        self.leader_follow_weight = 3.0
        self.max_speed = 1.5
    
    def get_leader_action(self, leader_id, waypoints, current_step):
        """Get action for a leader agent following waypoints."""
        # Simple waypoint following for leaders
        waypoint_index = (current_step // 50) % len(waypoints)
        target = waypoints[waypoint_index]
        
        return target
    
    def get_follower_action(self, follower_id, wrapper):
        """Get action for a follower using flocking rules."""
        sim = wrapper.get_simulator()
        
        # Get follower position and neighbors
        follower_pos = sim.get_agent_position(follower_id)
        neighbors = wrapper.get_neighbors(follower_id)
        
        # Initialize force vectors
        separation = Vector2(0, 0)
        alignment = Vector2(0, 0)
        cohesion = Vector2(0, 0)
        leader_follow = Vector2(0, 0)
        
        neighbor_count = 0
        leader_count = 0
        
        # Process neighbors
        for i in range(neighbors.shape[0]):
            if wrapper.isUsingObsMask():
                pos_x, pos_y, vel_x, vel_y, pv_x, pv_y, mask = neighbors[i]
                if mask < 0.5:  # Invalid neighbor
                    continue
            else:
                pos_x, pos_y, vel_x, vel_y, pv_x, pv_y = neighbors[i]
            
            # Neighbor position relative to follower
            neighbor_pos = Vector2(follower_pos.x() + pos_x, follower_pos.y() + pos_y)
            neighbor_vel = Vector2(vel_x, vel_y)
            
            # Distance to neighbor
            distance = np.sqrt(pos_x**2 + pos_y**2)
            
            if distance > 0.1:  # Avoid division by zero
                neighbor_count += 1
                
                # Separation: steer away from nearby neighbors
                if distance < 2.0:
                    sep_force = Vector2(-pos_x/distance, -pos_y/distance)
                    separation = separation + sep_force * (2.0 - distance)
                
                # Alignment: match neighbor velocities
                alignment = alignment + neighbor_vel
                
                # Cohesion: move toward neighbor center
                cohesion = cohesion + Vector2(pos_x, pos_y)
                
                # Leader following: strong attraction to leaders
                neighbor_id = self._find_neighbor_id(wrapper, follower_id, neighbor_pos)
                if neighbor_id in self.leaders:
                    leader_count += 1
                    leader_dir = Vector2(pos_x/distance, pos_y/distance)
                    leader_follow = leader_follow + leader_dir
        
        # Normalize forces
        if neighbor_count > 0:
            alignment = alignment * (1.0 / neighbor_count)
            cohesion = cohesion * (1.0 / neighbor_count)
        
        if leader_count > 0:
            leader_follow = leader_follow * (1.0 / leader_count)
        
        # Combine forces
        total_force = (separation * self.separation_weight +
                      alignment * self.alignment_weight +
                      cohesion * self.cohesion_weight +
                      leader_follow * self.leader_follow_weight)
        
        # Normalize to max speed
        force_magnitude = np.sqrt(total_force.x()**2 + total_force.y()**2)
        if force_magnitude > 0:
            total_force = total_force * (self.max_speed / force_magnitude)
        
        return total_force
    
    def _find_neighbor_id(self, wrapper, agent_id, neighbor_pos):
        """Find the ID of a neighbor based on position (approximate)."""
        sim = wrapper.get_simulator()
        min_distance = float('inf')
        closest_id = -1
        
        for i in range(sim.get_num_agents()):
            if i == agent_id:
                continue
            
            pos = sim.get_agent_position(i)
            distance = np.sqrt((pos.x() - neighbor_pos.x())**2 + 
                             (pos.y() - neighbor_pos.y())**2)
            
            if distance < min_distance:
                min_distance = distance
                closest_id = i
        
        return closest_id

def swarm_demo():
    """Demonstrate swarm behavior with leaders and followers."""
    
    num_agents = 12
    num_leaders = 2
    
    wrapper = RVO2RLWrapper(
        time_step=0.15,
        neighbor_dist=6.0,
        max_neighbors=8,
        radius=0.25,
        max_speed=1.5,
        mode=ObsMode.Cartesian,
        use_obs_mask=True
    )
    
    controller = SwarmController(num_agents, num_leaders)
    
    # Add agents in a cluster
    agent_ids = []
    np.random.seed(42)
    for i in range(num_agents):
        # Start in a loose cluster
        angle = np.random.uniform(0, 2*np.pi)
        radius = np.random.uniform(0.5, 2.0)
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        agent_id = wrapper.add_agent(Vector2(x, y))
        agent_ids.append(agent_id)
    
    # Dummy goals (will be overridden by swarm behavior)
    dummy_goals = [Vector2(0, 0) for _ in range(num_agents)]
    wrapper.set_goals(dummy_goals)
    wrapper.initialize()
    
    # Leader waypoints (path for leaders to follow)
    waypoints = [
        Vector2(5.0, 0.0),
        Vector2(5.0, 5.0),
        Vector2(0.0, 5.0),
        Vector2(-5.0, 5.0),
        Vector2(-5.0, 0.0),
        Vector2(-5.0, -5.0),
        Vector2(0.0, -5.0),
        Vector2(5.0, -5.0)
    ]
    
    print("Swarm Behavior Demonstration")
    print(f"Leaders: {controller.leaders}")
    print(f"Followers: {controller.followers}")
    print("=" * 40)
    
    for step in range(300):
        # Control each agent
        for i, agent_id in enumerate(agent_ids):
            if i in controller.leaders:
                # Leader behavior: follow waypoints
                target = controller.get_leader_action(i, waypoints, step)
                current_pos = wrapper.get_simulator().get_agent_position(agent_id)
                
                # Simple goal-seeking
                dx = target.x() - current_pos.x()
                dy = target.y() - current_pos.y()
                distance = np.sqrt(dx**2 + dy**2)
                
                if distance > 0.1:
                    preferred_vel = Vector2(dx/distance * 1.2, dy/distance * 1.2)
                else:
                    preferred_vel = Vector2(0, 0)
                
                wrapper.set_preferred_velocity(agent_id, preferred_vel)
            
            else:
                # Follower behavior: swarm rules
                swarm_velocity = controller.get_follower_action(agent_id, wrapper)
                wrapper.set_preferred_velocity(agent_id, swarm_velocity)
        
        # Step simulation
        wrapper.get_simulator().do_step()
        
        # Print status
        if step % 40 == 0:
            print(f"\\nStep {step}:")
            
            # Calculate swarm cohesion
            positions = []
            for agent_id in agent_ids:
                pos = wrapper.get_simulator().get_agent_position(agent_id)
                positions.append((pos.x(), pos.y()))
            
            center_x = np.mean([pos[0] for pos in positions])
            center_y = np.mean([pos[1] for pos in positions])
            
            # Calculate spread
            distances = [np.sqrt((pos[0] - center_x)**2 + (pos[1] - center_y)**2) 
                        for pos in positions]
            avg_spread = np.mean(distances)
            max_spread = np.max(distances)
            
            print(f"  Swarm center: ({center_x:.2f}, {center_y:.2f})")
            print(f"  Average spread: {avg_spread:.2f}")
            print(f"  Maximum spread: {max_spread:.2f}")
            
            # Leader positions
            for i, leader_id in enumerate(controller.leaders):
                pos = wrapper.get_simulator().get_agent_position(leader_id)
                print(f"  Leader {i}: ({pos.x():.2f}, {pos.y():.2f})")
    
    print("\\nSwarm demonstration completed!")

# Run the demo
swarm_demo()
```

## Scenario 3: Large-Scale Evacuation Simulation

Simulate emergency evacuation with hundreds of agents.

```python
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2
import numpy as np
import time

class EvacuationSimulation:
    """Large-scale evacuation simulation."""
    
    def __init__(self, num_agents=200, room_size=20.0):
        self.num_agents = num_agents
        self.room_size = room_size
        self.exit_positions = [
            Vector2(room_size/2, 0),      # Right exit
            Vector2(-room_size/2, 0),     # Left exit
            Vector2(0, room_size/2),      # Top exit
            Vector2(0, -room_size/2)      # Bottom exit
        ]
        
        # Performance optimized settings for large numbers of agents
        self.wrapper = RVO2RLWrapper(
            time_step=0.1,
            neighbor_dist=4.0,    # Reduced for performance
            max_neighbors=5,      # Reduced for performance
            radius=0.3,
            max_speed=2.0,
            mode=ObsMode.Cartesian
        )
        
        self.evacuated_count = 0
        self.evacuation_times = []
    
    def setup_scenario(self):
        """Set up the evacuation scenario."""
        # Add agents randomly throughout the room
        np.random.seed(42)
        self.agent_ids = []
        
        for i in range(self.num_agents):
            # Random position inside room (avoid walls)
            x = np.random.uniform(-self.room_size/2 + 1, self.room_size/2 - 1)
            y = np.random.uniform(-self.room_size/2 + 1, self.room_size/2 - 1)
            
            agent_id = self.wrapper.add_agent(Vector2(x, y))
            self.agent_ids.append(agent_id)
        
        # Add obstacles (interior walls, furniture, etc.)
        self._add_obstacles()
        
        # Assign exits to agents (closest exit strategy)
        self._assign_exits()
        
        self.wrapper.initialize()
        print(f"Evacuation simulation setup: {self.num_agents} agents")
    
    def _add_obstacles(self):
        """Add interior obstacles to make evacuation more realistic."""
        sim = self.wrapper.get_simulator()
        
        # Central pillar
        pillar = [
            Vector2(-1.5, -1.5), Vector2(1.5, -1.5),
            Vector2(1.5, 1.5), Vector2(-1.5, 1.5)
        ]
        sim.add_obstacle(pillar)
        
        # Some furniture/obstacles
        obstacles = [
            # Tables
            [Vector2(-8, -8), Vector2(-6, -8), Vector2(-6, -6), Vector2(-8, -6)],
            [Vector2(6, 6), Vector2(8, 6), Vector2(8, 8), Vector2(6, 8)],
            [Vector2(-8, 6), Vector2(-6, 6), Vector2(-6, 8), Vector2(-8, 8)],
            [Vector2(6, -8), Vector2(8, -8), Vector2(8, -6), Vector2(6, -6)]
        ]
        
        for obstacle in obstacles:
            sim.add_obstacle(obstacle)
        
        sim.process_obstacles()
    
    def _assign_exits(self):
        """Assign each agent to their nearest exit."""
        self.agent_exits = {}
        
        for agent_id in self.agent_ids:
            pos = self.wrapper.get_simulator().get_agent_position(agent_id)
            
            # Find closest exit
            min_distance = float('inf')
            closest_exit = self.exit_positions[0]
            
            for exit_pos in self.exit_positions:
                distance = np.sqrt((pos.x() - exit_pos.x())**2 + 
                                 (pos.y() - exit_pos.y())**2)
                if distance < min_distance:
                    min_distance = distance
                    closest_exit = exit_pos
            
            self.agent_exits[agent_id] = closest_exit
        
        # Set goals
        goals = [self.agent_exits[agent_id] for agent_id in self.agent_ids]
        self.wrapper.set_goals(goals)
    
    def run_simulation(self, max_steps=2000):
        """Run the evacuation simulation."""
        print("Starting evacuation simulation...")
        start_time = time.time()
        
        for step in range(max_steps):
            # Apply evacuation behavior
            self._update_agent_behaviors(step)
            
            # Step simulation
            self.wrapper.get_simulator().do_step()
            
            # Check for evacuated agents
            self._check_evacuations(step)
            
            # Progress reporting
            if step % 100 == 0:
                elapsed = time.time() - start_time
                print(f"Step {step}: {self.evacuated_count}/{self.num_agents} evacuated "
                      f"({elapsed:.1f}s elapsed)")
            
            # Check if all evacuated
            if self.evacuated_count >= self.num_agents:
                print(f"All agents evacuated at step {step}!")
                break
        
        self._print_results(time.time() - start_time)
    
    def _update_agent_behaviors(self, step):
        """Update agent behaviors for evacuation."""
        for agent_id in self.agent_ids:
            pos = self.wrapper.get_simulator().get_agent_position(agent_id)
            exit_pos = self.agent_exits[agent_id]
            
            # Check if agent has reached exit
            distance_to_exit = np.sqrt((pos.x() - exit_pos.x())**2 + 
                                     (pos.y() - exit_pos.y())**2)
            
            if distance_to_exit < 1.0:  # Near exit
                # Remove agent or mark as evacuated
                continue
            
            # Simple goal-seeking with urgency
            dx = exit_pos.x() - pos.x()
            dy = exit_pos.y() - pos.y()
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance > 0.1:
                # Increased urgency over time
                urgency_factor = min(2.0, 1.0 + step * 0.001)
                speed = min(2.0, urgency_factor)
                
                preferred_vel = Vector2((dx/distance) * speed, 
                                      (dy/distance) * speed)
                self.wrapper.set_preferred_velocity(agent_id, preferred_vel)
            else:
                self.wrapper.set_preferred_velocity(agent_id, Vector2(0, 0))
    
    def _check_evacuations(self, step):
        """Check which agents have successfully evacuated."""
        newly_evacuated = []
        
        for agent_id in self.agent_ids:
            pos = self.wrapper.get_simulator().get_agent_position(agent_id)
            
            # Check if agent is outside the room
            if (abs(pos.x()) > self.room_size/2 + 0.5 or 
                abs(pos.y()) > self.room_size/2 + 0.5):
                newly_evacuated.append(agent_id)
        
        # Record evacuation times
        for agent_id in newly_evacuated:
            if agent_id not in self.evacuation_times:
                self.evacuation_times.append((agent_id, step))
                self.evacuated_count += 1
    
    def _print_results(self, total_time):
        """Print simulation results."""
        print("\\n" + "="*50)
        print("EVACUATION SIMULATION RESULTS")
        print("="*50)
        print(f"Total agents: {self.num_agents}")
        print(f"Evacuated: {self.evacuated_count}")
        print(f"Evacuation rate: {self.evacuated_count/self.num_agents*100:.1f}%")
        print(f"Total simulation time: {total_time:.2f} seconds")
        
        if self.evacuation_times:
            times = [t[1] for t in self.evacuation_times]
            print(f"\\nEvacuation statistics:")
            print(f"  First evacuated: step {min(times)}")
            print(f"  Last evacuated: step {max(times)}")
            print(f"  Average evacuation time: {np.mean(times):.1f} steps")
            print(f"  Median evacuation time: {np.median(times):.1f} steps")

def run_evacuation_demo():
    """Run the evacuation demonstration."""
    # Test with different numbers of agents
    agent_counts = [100, 200]  # Reduced for demo
    
    for num_agents in agent_counts:
        print(f"\\n{'='*60}")
        print(f"EVACUATION DEMO: {num_agents} AGENTS")
        print(f"{'='*60}")
        
        simulation = EvacuationSimulation(num_agents=num_agents)
        simulation.setup_scenario()
        simulation.run_simulation(max_steps=1000)

# Run the demo
run_evacuation_demo()
```

## Scenario 4: Competitive Multi-Agent Game

Agents compete for limited resources while avoiding collisions.

```python
from rvo2_rl.rl import RVO2RLWrapper, ObsMode
from rvo2_rl.rvo2 import Vector2
import numpy as np

class CompetitiveGame:
    """Multi-agent competitive resource collection game."""
    
    def __init__(self, num_agents=8, num_resources=5):
        self.num_agents = num_agents
        self.num_resources = num_resources
        self.collected_resources = {}
        self.agent_scores = {i: 0 for i in range(num_agents)}
        
        self.wrapper = RVO2RLWrapper(
            time_step=0.1,
            neighbor_dist=8.0,
            max_neighbors=6,
            radius=0.4,
            max_speed=1.8,
            mode=ObsMode.Cartesian
        )
        
        self.resource_positions = []
        self.resource_collected = []
    
    def setup_game(self):
        """Set up the competitive game scenario."""
        # Add agents in a circle around the center
        self.agent_ids = []
        for i in range(self.num_agents):
            angle = 2 * np.pi * i / self.num_agents
            radius = 8.0
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            agent_id = self.wrapper.add_agent(Vector2(x, y))
            self.agent_ids.append(agent_id)
        
        # Place resources randomly in the center area
        np.random.seed(42)
        for i in range(self.num_resources):
            x = np.random.uniform(-4, 4)
            y = np.random.uniform(-4, 4)
            self.resource_positions.append(Vector2(x, y))
            self.resource_collected.append(False)
        
        # Initially, all agents target the center
        center = Vector2(0, 0)
        goals = [center for _ in range(self.num_agents)]
        self.wrapper.set_goals(goals)
        self.wrapper.initialize()
        
        print(f"Competitive game setup: {self.num_agents} agents, {self.num_resources} resources")
    
    def run_game(self, max_steps=500):
        """Run the competitive game."""
        print("Starting competitive resource collection game...")
        
        for step in range(max_steps):
            # Update agent strategies
            self._update_agent_strategies(step)
            
            # Step simulation
            self.wrapper.get_simulator().do_step()
            
            # Check resource collection
            self._check_resource_collection()
            
            # Respawn resources occasionally
            if step % 100 == 0 and step > 0:
                self._respawn_resources()
            
            # Progress reporting
            if step % 50 == 0:
                self._print_game_status(step)
            
            # Check if game should end
            if all(self.resource_collected):
                print(f"All resources collected at step {step}!")
                break
        
        self._print_final_results()
    
    def _update_agent_strategies(self, step):
        """Update each agent's strategy for resource collection."""
        for i, agent_id in enumerate(self.agent_ids):
            pos = self.wrapper.get_simulator().get_agent_position(agent_id)
            
            # Find nearest uncollected resource
            best_resource = self._find_best_resource(pos, agent_id)
            
            if best_resource is not None:
                # Move toward the best resource
                target = self.resource_positions[best_resource]
                dx = target.x() - pos.x()
                dy = target.y() - pos.y()
                distance = np.sqrt(dx**2 + dy**2)
                
                if distance > 0.1:
                    # Add some competitive behavior (move faster if others are nearby)
                    competition_factor = self._calculate_competition(pos, best_resource)
                    speed = min(1.8, 1.0 + competition_factor * 0.8)
                    
                    preferred_vel = Vector2((dx/distance) * speed, 
                                          (dy/distance) * speed)
                    self.wrapper.set_preferred_velocity(agent_id, preferred_vel)
                else:
                    self.wrapper.set_preferred_velocity(agent_id, Vector2(0, 0))
            else:
                # No resources available, patrol randomly
                if step % 30 == 0:  # Change direction every 3 seconds
                    angle = np.random.uniform(0, 2*np.pi)
                    preferred_vel = Vector2(np.cos(angle), np.sin(angle))
                    self.wrapper.set_preferred_velocity(agent_id, preferred_vel)
    
    def _find_best_resource(self, agent_pos, agent_id):
        """Find the best resource for an agent to target."""
        best_resource = None
        best_score = -1
        
        for i, (resource_pos, collected) in enumerate(zip(self.resource_positions, 
                                                         self.resource_collected)):
            if collected:
                continue
            
            # Distance to resource
            distance = np.sqrt((agent_pos.x() - resource_pos.x())**2 + 
                             (agent_pos.y() - resource_pos.y())**2)
            
            # Competition level (how many other agents are closer)
            competition = 0
            for other_id in self.agent_ids:
                if other_id == agent_id:
                    continue
                other_pos = self.wrapper.get_simulator().get_agent_position(other_id)
                other_distance = np.sqrt((other_pos.x() - resource_pos.x())**2 + 
                                       (other_pos.y() - resource_pos.y())**2)
                if other_distance < distance:
                    competition += 1
            
            # Score: prefer close resources with low competition
            score = 1.0 / (distance + 0.1) - competition * 0.2
            
            if score > best_score:
                best_score = score
                best_resource = i
        
        return best_resource
    
    def _calculate_competition(self, agent_pos, resource_index):
        """Calculate competition level for a resource."""
        resource_pos = self.resource_positions[resource_index]
        nearby_agents = 0
        
        for agent_id in self.agent_ids:
            other_pos = self.wrapper.get_simulator().get_agent_position(agent_id)
            distance_to_resource = np.sqrt((other_pos.x() - resource_pos.x())**2 + 
                                         (other_pos.y() - resource_pos.y())**2)
            
            if distance_to_resource < 3.0:  # Within competition radius
                nearby_agents += 1
        
        return min(1.0, nearby_agents / 3.0)  # Normalize to [0, 1]
    
    def _check_resource_collection(self):
        """Check if any agent has collected a resource."""
        for i, (resource_pos, collected) in enumerate(zip(self.resource_positions, 
                                                         self.resource_collected)):
            if collected:
                continue
            
            # Check if any agent is close enough to collect
            for agent_id in self.agent_ids:
                agent_pos = self.wrapper.get_simulator().get_agent_position(agent_id)
                distance = np.sqrt((agent_pos.x() - resource_pos.x())**2 + 
                                 (agent_pos.y() - resource_pos.y())**2)
                
                if distance < 0.6:  # Collection radius
                    # Agent collects the resource
                    self.resource_collected[i] = True
                    self.agent_scores[agent_id] += 1
                    print(f"  Agent {agent_id} collected resource {i}!")
                    break
    
    def _respawn_resources(self):
        """Respawn some collected resources."""
        respawn_count = 0
        for i in range(self.num_resources):
            if self.resource_collected[i] and np.random.random() < 0.3:  # 30% chance
                # Respawn at new random location
                x = np.random.uniform(-4, 4)
                y = np.random.uniform(-4, 4)
                self.resource_positions[i] = Vector2(x, y)
                self.resource_collected[i] = False
                respawn_count += 1
        
        if respawn_count > 0:
            print(f"  Respawned {respawn_count} resources")
    
    def _print_game_status(self, step):
        """Print current game status."""
        print(f"\\nStep {step}:")
        resources_left = sum(1 for collected in self.resource_collected if not collected)
        print(f"  Resources remaining: {resources_left}/{self.num_resources}")
        
        # Top 3 agents
        sorted_agents = sorted(self.agent_scores.items(), key=lambda x: x[1], reverse=True)
        print("  Top agents:")
        for i, (agent_id, score) in enumerate(sorted_agents[:3]):
            print(f"    {i+1}. Agent {agent_id}: {score} resources")
    
    def _print_final_results(self):
        """Print final game results."""
        print("\\n" + "="*40)
        print("COMPETITIVE GAME RESULTS")
        print("="*40)
        
        sorted_agents = sorted(self.agent_scores.items(), key=lambda x: x[1], reverse=True)
        
        print("Final standings:")
        for i, (agent_id, score) in enumerate(sorted_agents):
            print(f"  {i+1:2d}. Agent {agent_id}: {score} resources")
        
        winner = sorted_agents[0]
        print(f"\\nWinner: Agent {winner[0]} with {winner[1]} resources!")

def run_competitive_demo():
    """Run the competitive game demonstration."""
    game = CompetitiveGame(num_agents=6, num_resources=8)
    game.setup_game()
    game.run_game(max_steps=400)

# Run the demo
run_competitive_demo()
```

## Performance Considerations for Multi-Agent Scenarios

### Scaling Guidelines

| Scenario Type | Agents | max_neighbors | neighbor_dist | Performance |
|---------------|--------|---------------|---------------|-------------|
| Formation Control | 8-20 | 10-15 | 8-12 | Real-time |
| Swarm Behavior | 20-50 | 6-10 | 5-8 | Near real-time |
| Evacuation | 100-500 | 3-6 | 3-5 | Batch processing |
| Competitive | 10-30 | 6-8 | 6-10 | Real-time |

### Optimization Tips

1. **Reduce neighbor_dist**: For large crowds, agents don't need to see far
2. **Limit max_neighbors**: Fewer neighbors = faster computation
3. **Batch processing**: Process agents in groups for very large simulations
4. **Spatial filtering**: Only update agents in active areas

## Key Takeaways

1. **Formation Control**: Use centralized controllers to coordinate multiple agents
2. **Swarm Behavior**: Implement local rules (separation, alignment, cohesion) for emergent behavior
3. **Large-Scale Simulation**: Optimize parameters for performance vs. realism trade-offs
4. **Competitive Scenarios**: Add game mechanics on top of collision avoidance

## Next Steps

- Explore [Advanced Examples](advanced.md) for LiDAR integration and complex environments
- Check [Performance Optimization](../technical/performance.md) for scaling to thousands of agents
- See [RL Extensions API](../api/rl_extensions.md) for implementing custom behaviors
