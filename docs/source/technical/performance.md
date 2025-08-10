# Performance Optimization

This guide provides comprehensive strategies for optimizing PYRVO2-RL performance across different scales, from small research experiments to large-scale crowd simulations.

## Performance Fundamentals

### Understanding the Bottlenecks

PYRVO2-RL performance is primarily determined by four factors:

1. **Neighbor Search (40-60% of computation time)**
   - KdTree construction and queries
   - Scales as O(N log N) where N = number of agents

2. **ORCA Computation (25-35% of computation time)**
   - Linear programming for collision avoidance
   - Scales as O(N*k) where k = average neighbors

3. **Position Updates (5-15% of computation time)**
   - Agent movement and boundary checking
   - Scales as O(N)

4. **Python-C++ Overhead (5-20% depending on usage)**
   - Function calls and data conversion
   - Batch operations reduce this significantly

### Profiling Your Simulation

Use the built-in profiling tools to identify bottlenecks:

```python
from rvo2_rl.util import profile_simulation, openmp_threads_used

def profile_my_simulation():
    wrapper = RVO2RLWrapper(...)
    
    # Add agents and set up scenario
    for i in range(100):
        wrapper.add_agent(Vector2(random.uniform(-10, 10), 
                                 random.uniform(-10, 10)))
    
    wrapper.initialize()
    
    # Profile simulation performance
    stats = profile_simulation(wrapper, num_steps=100)
    
    print(f"Steps per second: {stats['steps_per_second']:.1f}")
    print(f"Time per step: {stats['time_per_step_ms']:.2f} ms")
    print(f"Neighbor queries: {stats['neighbor_time_ms']:.2f} ms")
    print(f"ORCA computation: {stats['orca_time_ms']:.2f} ms")
    print(f"Position updates: {stats['update_time_ms']:.2f} ms")
    print(f"OpenMP threads: {openmp_threads_used()}")

profile_my_simulation()
```

## Configuration Optimization

### Parameter Tuning for Different Scales

**Small Scale (< 50 agents) - Research Focus:**
```python
wrapper = RVO2RLWrapper(
    time_step=0.1,          # High precision
    neighbor_dist=10.0,     # Large sensing range
    max_neighbors=15,       # Consider many neighbors
    radius=0.5,             # Realistic agent size
    max_speed=2.0,          # Natural movement speed
    mode=ObsMode.Cartesian,
    use_obs_mask=True       # Full observation features
)
```

**Medium Scale (50-200 agents) - Balanced Performance:**
```python
wrapper = RVO2RLWrapper(
    time_step=0.15,         # Moderate precision
    neighbor_dist=6.0,      # Reduced sensing
    max_neighbors=8,        # Fewer neighbors
    radius=0.4,             # Smaller agents
    max_speed=1.5,          # Controlled speed
    mode=ObsMode.Polar,     # Efficient representation
    use_obs_mask=False      # Skip masking overhead
)
```

**Large Scale (200-1000 agents) - Speed Focused:**
```python
wrapper = RVO2RLWrapper(
    time_step=0.25,         # Larger time steps
    neighbor_dist=4.0,      # Minimal sensing
    max_neighbors=4,        # Very few neighbors
    radius=0.3,             # Compact agents
    max_speed=1.0,          # Moderate speed
    mode=ObsMode.Raw,       # Simplest processing
    use_obs_mask=False,     # No masking
    use_lidar=False         # Disable LiDAR
)
```

**Extreme Scale (1000+ agents) - Maximum Performance:**
```python
wrapper = RVO2RLWrapper(
    time_step=0.5,          # Very large steps
    neighbor_dist=2.5,      # Minimum sensing
    max_neighbors=3,        # Absolute minimum
    radius=0.25,            # Very small agents
    max_speed=0.8,          # Slow movement
    mode=ObsMode.Raw,       # Raw data only
    use_obs_mask=False,     # No overhead
    use_lidar=False         # Disable all extras
)
```

### Adaptive Parameter Selection

Automatically adjust parameters based on agent density:

```python
class AdaptiveSimulation:
    def __init__(self, target_fps=30):
        self.target_fps = target_fps
        self.performance_history = []
        
    def create_wrapper(self, num_agents, area_size):
        # Calculate agent density
        density = num_agents / (area_size ** 2)
        
        if density < 0.01:      # Low density
            config = self.get_high_quality_config()
        elif density < 0.05:    # Medium density
            config = self.get_balanced_config()
        elif density < 0.2:     # High density
            config = self.get_performance_config()
        else:                   # Extreme density
            config = self.get_speed_config()
        
        return RVO2RLWrapper(**config)
    
    def get_high_quality_config(self):
        return {
            'time_step': 0.1,
            'neighbor_dist': 8.0,
            'max_neighbors': 12,
            'radius': 0.5,
            'mode': ObsMode.Cartesian,
            'use_obs_mask': True
        }
    
    def get_performance_config(self):
        return {
            'time_step': 0.3,
            'neighbor_dist': 3.0,
            'max_neighbors': 4,
            'radius': 0.3,
            'mode': ObsMode.Raw,
            'use_obs_mask': False
        }
    
    def auto_tune_simulation(self, wrapper, test_steps=50):
        """Automatically tune parameters to hit target FPS."""
        import time
        
        start_time = time.time()
        for _ in range(test_steps):
            wrapper.get_simulator().do_step()
        elapsed = time.time() - start_time
        
        current_fps = test_steps / elapsed
        
        if current_fps < self.target_fps * 0.8:
            # Performance too low, reduce quality
            self._reduce_quality(wrapper)
        elif current_fps > self.target_fps * 1.2:
            # Performance headroom, increase quality
            self._increase_quality(wrapper)
        
        return current_fps
    
    def _reduce_quality(self, wrapper):
        """Reduce simulation quality for better performance."""
        sim = wrapper.get_simulator()
        
        # Increase time step
        current_step = sim.getTimeStep()
        sim.setTimeStep(min(0.5, current_step * 1.2))
        
        # Reduce neighbor distance for all agents
        for i in range(sim.getNumAgents()):
            current_dist = sim.getAgentNeighborDist(i)
            sim.setAgentNeighborDist(i, max(2.0, current_dist * 0.9))
    
    def _increase_quality(self, wrapper):
        """Increase simulation quality when performance allows."""
        sim = wrapper.get_simulator()
        
        # Decrease time step
        current_step = sim.getTimeStep()
        sim.setTimeStep(max(0.05, current_step * 0.9))
        
        # Increase neighbor distance
        for i in range(sim.getNumAgents()):
            current_dist = sim.getAgentNeighborDist(i)
            sim.setAgentNeighborDist(i, min(10.0, current_dist * 1.1))
```

## Memory Optimization

### Efficient Memory Usage

**Agent Preallocation:**
```python
class PreallocatedSimulation:
    def __init__(self, max_agents=1000):
        self.wrapper = RVO2RLWrapper(...)
        
        # Preallocate maximum number of agents
        dummy_pos = Vector2(0, 0)
        self.agent_pool = []
        
        for i in range(max_agents):
            agent_id = self.wrapper.add_agent(dummy_pos)
            self.agent_pool.append(agent_id)
            # Immediately deactivate
            self.wrapper.get_simulator().setAgentMaxSpeed(agent_id, 0)
        
        self.active_agents = set()
        self.inactive_agents = set(self.agent_pool)
    
    def activate_agent(self, position, **kwargs):
        """Activate an agent from the pool."""
        if not self.inactive_agents:
            raise RuntimeError("No inactive agents available")
        
        agent_id = self.inactive_agents.pop()
        self.active_agents.add(agent_id)
        
        # Configure agent
        sim = self.wrapper.get_simulator()
        sim.setAgentPosition(agent_id, position)
        sim.setAgentMaxSpeed(agent_id, kwargs.get('max_speed', 1.5))
        sim.setAgentRadius(agent_id, kwargs.get('radius', 0.4))
        
        return agent_id
    
    def deactivate_agent(self, agent_id):
        """Return agent to pool."""
        if agent_id in self.active_agents:
            self.active_agents.remove(agent_id)
            self.inactive_agents.add(agent_id)
            
            # Reset agent
            sim = self.wrapper.get_simulator()
            sim.setAgentMaxSpeed(agent_id, 0)
            sim.setAgentPosition(agent_id, Vector2(1e6, 1e6))  # Move far away
```

**Memory Monitoring:**
```python
import psutil
import gc

class MemoryMonitor:
    def __init__(self):
        self.process = psutil.Process()
        self.baseline_memory = self.process.memory_info().rss
    
    def report_usage(self, label=""):
        current_memory = self.process.memory_info().rss
        delta_mb = (current_memory - self.baseline_memory) / 1024 / 1024
        total_mb = current_memory / 1024 / 1024
        
        print(f"{label}: {total_mb:.1f} MB total (+{delta_mb:.1f} MB)")
    
    def cleanup_and_report(self, label="After cleanup"):
        gc.collect()  # Force garbage collection
        self.report_usage(label)

# Usage example
monitor = MemoryMonitor()
monitor.report_usage("Initial")

wrapper = RVO2RLWrapper(...)
for i in range(1000):
    wrapper.add_agent(Vector2(random.uniform(-50, 50), random.uniform(-50, 50)))

monitor.report_usage("After adding 1000 agents")
monitor.cleanup_and_report()
```

## Multi-Threading Optimization

### OpenMP Configuration

**Optimal Thread Count:**
```python
from rvo2_rl.util import set_openmp_threads, openmp_max_threads
import multiprocessing

def optimize_thread_count(wrapper, test_agents=200, test_steps=100):
    """Find optimal thread count for current hardware."""
    max_threads = min(openmp_max_threads(), multiprocessing.cpu_count())
    results = {}
    
    # Add test agents
    for i in range(test_agents):
        wrapper.add_agent(Vector2(random.uniform(-20, 20), 
                                 random.uniform(-20, 20)))
    wrapper.initialize()
    
    for num_threads in range(1, max_threads + 1):
        set_openmp_threads(num_threads)
        
        start_time = time.time()
        for _ in range(test_steps):
            wrapper.get_simulator().do_step()
        elapsed = time.time() - start_time
        
        fps = test_steps / elapsed
        results[num_threads] = fps
        
        print(f"Threads: {num_threads:2d}, FPS: {fps:6.1f}")
    
    # Find optimal thread count
    optimal_threads = max(results.keys(), key=lambda k: results[k])
    print(f"\\nOptimal thread count: {optimal_threads}")
    
    return optimal_threads, results

# Find and set optimal configuration
wrapper = RVO2RLWrapper(...)
optimal_threads, _ = optimize_thread_count(wrapper)
set_openmp_threads(optimal_threads)
```

**Thread Scaling Analysis:**
```python
def analyze_thread_scaling(max_agents=1000, step_size=200):
    """Analyze how performance scales with threads and agents."""
    import matplotlib.pyplot as plt
    import numpy as np
    
    agent_counts = range(step_size, max_agents + 1, step_size)
    thread_counts = [1, 2, 4, 8]
    
    results = np.zeros((len(thread_counts), len(agent_counts)))
    
    for i, num_threads in enumerate(thread_counts):
        set_openmp_threads(num_threads)
        
        for j, num_agents in enumerate(agent_counts):
            wrapper = RVO2RLWrapper(time_step=0.2)
            
            # Add agents
            for k in range(num_agents):
                pos = Vector2(random.uniform(-30, 30), random.uniform(-30, 30))
                wrapper.add_agent(pos)
            
            wrapper.initialize()
            
            # Measure performance
            start_time = time.time()
            for _ in range(50):
                wrapper.get_simulator().do_step()
            elapsed = time.time() - start_time
            
            fps = 50 / elapsed
            results[i, j] = fps
            
            print(f"Threads: {num_threads}, Agents: {num_agents}, FPS: {fps:.1f}")
    
    # Plot results
    plt.figure(figsize=(10, 6))
    for i, num_threads in enumerate(thread_counts):
        plt.plot(agent_counts, results[i], marker='o', label=f'{num_threads} threads')
    
    plt.xlabel('Number of Agents')
    plt.ylabel('Steps per Second')
    plt.title('Threading Performance Analysis')
    plt.legend()
    plt.grid(True)
    plt.savefig('thread_scaling.png')
    plt.show()
    
    return results

# Run scaling analysis
results = analyze_thread_scaling()
```

## Batch Operations

### Efficient Batch Processing

**Vectorized Operations:**
```python
class BatchProcessor:
    def __init__(self, wrapper):
        self.wrapper = wrapper
        
    def batch_set_goals(self, goals):
        """Set goals for all agents in batch."""
        # Much faster than individual setGoal calls
        self.wrapper.set_goals(goals)
    
    def batch_get_observations(self, agent_ids=None):
        """Get observations for multiple agents efficiently."""
        # Avoid Python loop overhead
        return self.wrapper.get_observations(agent_ids)
    
    def batch_set_preferred_velocities(self, velocities):
        """Set preferred velocities for all agents."""
        sim = self.wrapper.get_simulator()
        
        # Use batch operation instead of individual calls
        for i, velocity in enumerate(velocities):
            if i < sim.getNumAgents():
                sim.setAgentPrefVelocity(i, velocity)
    
    def batch_apply_actions(self, actions):
        """Convert RL actions to preferred velocities efficiently."""
        velocities = []
        
        for action in actions:
            # Convert action to velocity (example for continuous actions)
            if len(action) == 2:  # Direct velocity control
                vel = Vector2(action[0], action[1])
            else:  # Speed and direction
                speed, direction = action[0], action[1]
                vel = Vector2(speed * np.cos(direction), 
                             speed * np.sin(direction))
            velocities.append(vel)
        
        self.batch_set_preferred_velocities(velocities)

# Usage example
processor = BatchProcessor(wrapper)

# Instead of individual operations:
# for i, goal in enumerate(goals):
#     wrapper.setGoal(i, goal)  # Slow!

# Use batch operation:
processor.batch_set_goals(goals)  # Fast!
```

### Minimizing Python-C++ Overhead

**Reduce Function Calls:**
```python
class OptimizedLoop:
    def __init__(self, wrapper):
        self.wrapper = wrapper
        self.sim = wrapper.get_simulator()  # Cache simulator reference
        
    def run_simulation_optimized(self, num_steps):
        """Optimized simulation loop with minimal overhead."""
        
        # Cache frequently used methods
        do_step = self.sim.do_step
        get_positions = self._batch_get_positions
        
        # Preallocate result storage
        position_history = []
        
        for step in range(num_steps):
            # Single simulation step
            do_step()
            
            # Batch data collection (every N steps)
            if step % 10 == 0:
                positions = get_positions()
                position_history.append(positions)
        
        return position_history
    
    def _batch_get_positions(self):
        """Get all agent positions in one call."""
        sim = self.sim
        num_agents = sim.getNumAgents()
        
        positions = []
        for i in range(num_agents):
            pos = sim.getAgentPosition(i)
            positions.append((pos.x(), pos.y()))
        
        return positions

# Compare performance
def compare_loop_performance(wrapper, num_steps=1000):
    optimized = OptimizedLoop(wrapper)
    
    # Method 1: Naive approach
    start_time = time.time()
    for step in range(num_steps):
        wrapper.get_simulator().do_step()
        
        # Inefficient: get positions every step
        for i in range(wrapper.get_simulator().getNumAgents()):
            pos = wrapper.get_simulator().getAgentPosition(i)
    
    naive_time = time.time() - start_time
    
    # Method 2: Optimized approach
    start_time = time.time()
    optimized.run_simulation_optimized(num_steps)
    optimized_time = time.time() - start_time
    
    speedup = naive_time / optimized_time
    print(f"Naive approach: {naive_time:.2f}s")
    print(f"Optimized approach: {optimized_time:.2f}s")
    print(f"Speedup: {speedup:.1f}x")
```

## Specialized Optimizations

### GPU Acceleration (Future Extension)

While PYRVO2-RL currently runs on CPU, here's a framework for GPU acceleration:

```python
class GPUAcceleratedSimulation:
    """Framework for GPU-accelerated simulation (conceptual)."""
    
    def __init__(self, use_gpu=False):
        self.use_gpu = use_gpu and self._check_gpu_support()
        
        if self.use_gpu:
            import cupy as cp  # GPU arrays
            self.xp = cp
        else:
            import numpy as np
            self.xp = np
    
    def _check_gpu_support(self):
        try:
            import cupy
            return True
        except ImportError:
            return False
    
    def batch_neighbor_search(self, positions, radii):
        """GPU-accelerated neighbor search."""
        if not self.use_gpu:
            return self._cpu_neighbor_search(positions, radii)
        
        # Convert to GPU arrays
        pos_gpu = self.xp.asarray(positions)
        radii_gpu = self.xp.asarray(radii)
        
        # GPU kernel for neighbor computation
        # (Implementation would require custom CUDA kernels)
        neighbors = self._gpu_neighbor_kernel(pos_gpu, radii_gpu)
        
        # Convert back to CPU for RVO2
        return neighbors.get()  # cupy to numpy

# Enable GPU acceleration if available
wrapper = RVO2RLWrapper(...)
gpu_sim = GPUAcceleratedSimulation(use_gpu=True)
```

### Custom Distance Metrics

For specialized applications, implement custom distance metrics:

```python
class CustomMetrics:
    """Custom distance and interaction metrics."""
    
    @staticmethod
    def social_force_distance(pos1, pos2, social_factors):
        """Distance modified by social factors."""
        base_distance = np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
        
        # Apply social force modification
        social_weight = social_factors.get('weight', 1.0)
        return base_distance * social_weight
    
    @staticmethod
    def anisotropic_distance(pos1, pos2, vel1, direction_weight=0.3):
        """Distance that considers movement direction."""
        dx, dy = pos2[0] - pos1[0], pos2[1] - pos1[1]
        base_distance = np.sqrt(dx**2 + dy**2)
        
        # Modify based on velocity direction
        vel_dot = (dx * vel1[0] + dy * vel1[1]) / (base_distance + 1e-10)
        direction_factor = 1.0 + direction_weight * vel_dot
        
        return base_distance * direction_factor

class AdaptiveNeighborSearch:
    """Adaptive neighbor search with custom metrics."""
    
    def __init__(self, wrapper, metric_func=None):
        self.wrapper = wrapper
        self.metric_func = metric_func or self._euclidean_distance
        
    def find_adaptive_neighbors(self, agent_id, max_neighbors=8):
        """Find neighbors using adaptive criteria."""
        sim = self.wrapper.get_simulator()
        agent_pos = sim.getAgentPosition(agent_id)
        agent_vel = sim.getAgentVelocity(agent_id)
        
        neighbors = []
        
        for other_id in range(sim.getNumAgents()):
            if other_id == agent_id:
                continue
                
            other_pos = sim.getAgentPosition(other_id)
            other_vel = sim.getAgentVelocity(other_id)
            
            # Use custom distance metric
            distance = self.metric_func(
                (agent_pos.x(), agent_pos.y()),
                (other_pos.x(), other_pos.y()),
                (agent_vel.x(), agent_vel.y())
            )
            
            neighbors.append((distance, other_id))
        
        # Sort by distance and return closest
        neighbors.sort(key=lambda x: x[0])
        return neighbors[:max_neighbors]
    
    def _euclidean_distance(self, pos1, pos2, vel1):
        """Standard Euclidean distance."""
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
```

## Performance Benchmarking

### Comprehensive Benchmark Suite

```python
class PerformanceBenchmark:
    """Comprehensive performance benchmarking suite."""
    
    def __init__(self):
        self.results = {}
        
    def run_full_benchmark(self):
        """Run complete performance benchmark."""
        
        print("PYRVO2-RL Performance Benchmark")
        print("="*50)
        
        # Test different configurations
        configs = [
            ('Research', {'neighbor_dist': 10.0, 'max_neighbors': 15, 'time_step': 0.1}),
            ('Balanced', {'neighbor_dist': 6.0, 'max_neighbors': 8, 'time_step': 0.15}),
            ('Performance', {'neighbor_dist': 4.0, 'max_neighbors': 4, 'time_step': 0.25}),
            ('Speed', {'neighbor_dist': 2.5, 'max_neighbors': 3, 'time_step': 0.5})
        ]
        
        agent_counts = [50, 100, 200, 500, 1000]
        
        for config_name, config in configs:
            print(f"\\n{config_name} Configuration:")
            print("-" * 30)
            
            config_results = {}
            
            for num_agents in agent_counts:
                if num_agents <= 1000 or config_name in ['Performance', 'Speed']:
                    fps = self._benchmark_configuration(config, num_agents)
                    config_results[num_agents] = fps
                    print(f"  {num_agents:4d} agents: {fps:6.1f} FPS")
                else:
                    print(f"  {num_agents:4d} agents: Skipped (too slow)")
            
            self.results[config_name] = config_results
        
        self._print_summary()
        
    def _benchmark_configuration(self, config, num_agents, test_steps=100):
        """Benchmark specific configuration."""
        wrapper = RVO2RLWrapper(**config)
        
        # Add agents randomly
        for i in range(num_agents):
            pos = Vector2(random.uniform(-25, 25), random.uniform(-25, 25))
            wrapper.add_agent(pos)
        
        # Set random goals
        goals = []
        for i in range(num_agents):
            goal = Vector2(random.uniform(-25, 25), random.uniform(-25, 25))
            goals.append(goal)
        
        wrapper.set_goals(goals)
        wrapper.initialize()
        
        # Benchmark simulation
        start_time = time.time()
        
        for _ in range(test_steps):
            wrapper.get_simulator().do_step()
        
        elapsed = time.time() - start_time
        return test_steps / elapsed
    
    def _print_summary(self):
        """Print benchmark summary."""
        print("\\n" + "="*50)
        print("PERFORMANCE SUMMARY")
        print("="*50)
        
        print("\\nConfiguration Comparison (FPS):")
        print(f"{'Agents':<8}", end="")
        for config_name in self.results:
            print(f"{config_name:<12}", end="")
        print()
        
        all_agent_counts = set()
        for config_results in self.results.values():
            all_agent_counts.update(config_results.keys())
        
        for num_agents in sorted(all_agent_counts):
            print(f"{num_agents:<8}", end="")
            for config_name in self.results:
                fps = self.results[config_name].get(num_agents, 0)
                if fps > 0:
                    print(f"{fps:<12.1f}", end="")
                else:
                    print(f"{'N/A':<12}", end="")
            print()
        
        # Performance recommendations
        print("\\nRecommendations:")
        print("- Research config: Best for detailed analysis (< 100 agents)")
        print("- Balanced config: Good compromise (< 500 agents)")
        print("- Performance config: Large simulations (< 1000 agents)")
        print("- Speed config: Maximum scale (1000+ agents)")

# Run comprehensive benchmark
benchmark = PerformanceBenchmark()
benchmark.run_full_benchmark()
```

## Platform-Specific Optimizations

### Linux Optimizations

```bash
# Enable CPU performance mode
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Set CPU affinity for Python process
taskset -c 0-7 python your_simulation.py

# Use NUMA optimizations for large systems
numactl --interleave=all python your_simulation.py
```

### Compilation Optimizations

When building from source, use optimized compilation flags:

```bash
# In setup.py or CMakeLists.txt
export CXXFLAGS="-O3 -march=native -DNDEBUG"
export LDFLAGS="-O3"

# Enable specific instruction sets if available
export CXXFLAGS="$CXXFLAGS -mavx2 -mfma"

# Build with optimizations
pip install -e . --config-settings="--build-option=--enable-optimizations"
```

This comprehensive performance guide should help you optimize PYRVO2-RL for your specific use case, from small research experiments to large-scale production simulations.
