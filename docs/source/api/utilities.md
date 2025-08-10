# Utilities API Reference

The `rvo2_rl.util` module provides utility functions for performance monitoring and system information.

## OpenMP Support

PYRVO2-RL uses OpenMP for parallel computation when available, significantly improving performance for large numbers of agents.

### Thread Information

```python
from rvo2_rl.util import openmp_max_threads, openmp_threads_used

# Check OpenMP availability and configuration
max_threads = openmp_max_threads()
actual_threads = openmp_threads_used()

print(f"OpenMP max threads: {max_threads}")
print(f"OpenMP threads in use: {actual_threads}")
```

#### Functions

```python
def openmp_max_threads() -> int:
    """Get the maximum number of OpenMP threads available.
    
    Returns:
        Maximum number of threads available to OpenMP.
        Returns 1 if OpenMP is not available.
    """

def openmp_threads_used() -> int:
    """Get the number of OpenMP threads actually being used.
    
    This function executes a parallel region to count the 
    active threads, providing the actual parallelization level.
    
    Returns:
        Number of threads actively used by OpenMP.
        Returns 1 if OpenMP is not available.
    """
```

## Usage Examples

### Performance Monitoring

```python
import time
import numpy as np
from rvo2_rl.rl import RVO2RLWrapper
from rvo2_rl.rvo2 import Vector2
from rvo2_rl.util import openmp_max_threads, openmp_threads_used

def benchmark_simulation(num_agents_list=[10, 50, 100, 200]):
    """Benchmark simulation performance with different agent counts."""
    
    print(f"OpenMP Configuration:")
    print(f"  Max threads: {openmp_max_threads()}")
    print(f"  Used threads: {openmp_threads_used()}")
    print()
    
    results = []
    
    for num_agents in num_agents_list:
        print(f"Benchmarking {num_agents} agents...")
        
        # Create simulation
        wrapper = RVO2RLWrapper(
            time_step=0.1,
            neighbor_dist=10.0,
            max_neighbors=5,
            radius=0.5,
            max_speed=2.0
        )
        
        # Add agents in random positions
        np.random.seed(42)  # Reproducible results
        for i in range(num_agents):
            x = np.random.uniform(-20, 20)
            y = np.random.uniform(-20, 20)
            wrapper.add_agent(Vector2(x, y))
        
        # Set random goals
        goals = []
        for i in range(num_agents):
            x = np.random.uniform(-20, 20)
            y = np.random.uniform(-20, 20)
            goals.append(Vector2(x, y))
        wrapper.set_goals(goals)
        wrapper.initialize()
        
        # Benchmark simulation steps
        num_steps = 100
        start_time = time.time()
        
        for step in range(num_steps):
            wrapper.set_preferred_velocities()
            wrapper.get_simulator().do_step()
        
        elapsed_time = time.time() - start_time
        steps_per_second = num_steps / elapsed_time
        agents_steps_per_second = num_agents * steps_per_second
        
        results.append({
            'agents': num_agents,
            'steps_per_second': steps_per_second,
            'agents_steps_per_second': agents_steps_per_second,
            'time_per_step': elapsed_time / num_steps
        })
        
        print(f"  {steps_per_second:.1f} steps/sec")
        print(f"  {agents_steps_per_second:.0f} agent-steps/sec")
        print(f"  {elapsed_time/num_steps*1000:.2f} ms/step")
        print()
    
    return results

# Run benchmark
if __name__ == "__main__":
    results = benchmark_simulation()
    
    print("Performance Summary:")
    print("Agents | Steps/sec | Agent-steps/sec | ms/step")
    print("-" * 50)
    for result in results:
        print(f"{result['agents']:6d} | {result['steps_per_second']:8.1f} | "
              f"{result['agents_steps_per_second']:14.0f} | "
              f"{result['time_per_step']*1000:6.2f}")
```

### System Information

```python
import platform
import psutil  # Install with: pip install psutil
from rvo2_rl.util import openmp_max_threads, openmp_threads_used

def system_info():
    """Display comprehensive system information for performance analysis."""
    
    print("System Information for PYRVO2-RL")
    print("=" * 40)
    
    # Basic system info
    print(f"Platform: {platform.platform()}")
    print(f"Processor: {platform.processor()}")
    print(f"CPU count: {psutil.cpu_count(logical=False)} physical, "
          f"{psutil.cpu_count(logical=True)} logical")
    
    # Memory info
    memory = psutil.virtual_memory()
    print(f"Memory: {memory.total // (1024**3)} GB total, "
          f"{memory.available // (1024**3)} GB available")
    
    # OpenMP info
    print(f"OpenMP max threads: {openmp_max_threads()}")
    print(f"OpenMP used threads: {openmp_threads_used()}")
    
    # Performance recommendations
    print()
    print("Performance Recommendations:")
    max_threads = openmp_max_threads()
    if max_threads == 1:
        print("⚠ OpenMP not available - consider recompiling with OpenMP support")
    elif max_threads >= 4:
        print("✓ Good OpenMP support available")
    else:
        print("⚠ Limited parallelization available")
    
    if memory.available < 2 * (1024**3):
        print("⚠ Low memory - consider smaller simulations")
    else:
        print("✓ Sufficient memory available")

if __name__ == "__main__":
    system_info()
```

### Performance Optimization

```python
import os
from rvo2_rl.util import openmp_max_threads, openmp_threads_used

def optimize_performance():
    """Set optimal performance parameters based on system capabilities."""
    
    max_threads = openmp_max_threads()
    
    print(f"Detected {max_threads} OpenMP threads")
    
    if max_threads > 1:
        # OpenMP is available
        if max_threads >= 8:
            # High-end system
            print("High-performance system detected")
            recommended_agents = 500
            recommended_neighbors = 10
        elif max_threads >= 4:
            # Mid-range system
            print("Mid-range system detected")
            recommended_agents = 200
            recommended_neighbors = 8
        else:
            # Low-end system with OpenMP
            print("Basic system with OpenMP detected")
            recommended_agents = 100
            recommended_neighbors = 5
        
        # Set OpenMP environment variables for optimal performance
        os.environ['OMP_NUM_THREADS'] = str(max_threads)
        os.environ['OMP_SCHEDULE'] = 'dynamic'
        os.environ['OMP_PROC_BIND'] = 'true'
        
    else:
        # No OpenMP
        print("No OpenMP support detected")
        recommended_agents = 50
        recommended_neighbors = 3
    
    print(f"Recommended settings:")
    print(f"  Max agents: {recommended_agents}")
    print(f"  Max neighbors: {recommended_neighbors}")
    print(f"  Time step: 0.1-0.25 (smaller for more accuracy)")
    
    return {
        'max_agents': recommended_agents,
        'max_neighbors': recommended_neighbors,
        'time_step': 0.1 if max_threads >= 4 else 0.25
    }

# Use in your application
settings = optimize_performance()

# Apply settings to your simulation
wrapper = RVO2RLWrapper(
    time_step=settings['time_step'],
    max_neighbors=settings['max_neighbors'],
    # ... other parameters
)
```

### Thread Scaling Analysis

```python
import os
import time
import numpy as np
from rvo2_rl.rl import RVO2RLWrapper
from rvo2_rl.rvo2 import Vector2
from rvo2_rl.util import openmp_max_threads, openmp_threads_used

def thread_scaling_benchmark():
    """Analyze how performance scales with different thread counts."""
    
    max_threads = openmp_max_threads()
    if max_threads <= 1:
        print("OpenMP not available - cannot test thread scaling")
        return
    
    # Test different thread counts
    thread_counts = [1, 2, 4, max_threads] if max_threads >= 4 else [1, max_threads]
    thread_counts = list(set(thread_counts))  # Remove duplicates
    thread_counts.sort()
    
    num_agents = 200
    num_steps = 50
    
    print(f"Thread Scaling Benchmark ({num_agents} agents, {num_steps} steps)")
    print("=" * 60)
    
    baseline_time = None
    
    for thread_count in thread_counts:
        # Set thread count
        os.environ['OMP_NUM_THREADS'] = str(thread_count)
        
        # Create fresh simulation
        wrapper = RVO2RLWrapper(
            time_step=0.1,
            neighbor_dist=8.0,
            max_neighbors=5,
            radius=0.4,
            max_speed=1.5
        )
        
        # Add agents
        np.random.seed(42)  # Consistent setup
        for i in range(num_agents):
            x = np.random.uniform(-15, 15)
            y = np.random.uniform(-15, 15)
            wrapper.add_agent(Vector2(x, y))
        
        goals = [Vector2(np.random.uniform(-15, 15), 
                        np.random.uniform(-15, 15)) 
                for _ in range(num_agents)]
        wrapper.set_goals(goals)
        wrapper.initialize()
        
        # Benchmark
        start_time = time.time()
        for step in range(num_steps):
            wrapper.set_preferred_velocities()
            wrapper.get_simulator().do_step()
        elapsed_time = time.time() - start_time
        
        # Verify thread usage
        used_threads = openmp_threads_used()
        
        if baseline_time is None:
            baseline_time = elapsed_time
            speedup = 1.0
        else:
            speedup = baseline_time / elapsed_time
        
        efficiency = speedup / thread_count * 100
        
        print(f"Threads: {thread_count:2d} | Used: {used_threads:2d} | "
              f"Time: {elapsed_time:.3f}s | Speedup: {speedup:.2f}x | "
              f"Efficiency: {efficiency:.1f}%")
    
    # Reset to default
    if 'OMP_NUM_THREADS' in os.environ:
        del os.environ['OMP_NUM_THREADS']

if __name__ == "__main__":
    thread_scaling_benchmark()
```

## Performance Tips

### OpenMP Optimization

1. **Thread Count**: By default, OpenMP uses all available CPU cores. You can control this:
   ```python
   import os
   os.environ['OMP_NUM_THREADS'] = '4'  # Use 4 threads
   ```

2. **Scheduling**: For best performance with varying workloads:
   ```python
   os.environ['OMP_SCHEDULE'] = 'dynamic,1'
   ```

3. **Thread Affinity**: Bind threads to cores for better cache performance:
   ```python
   os.environ['OMP_PROC_BIND'] = 'true'
   ```

### Memory Optimization

1. **Agent Count vs. Memory**: Each agent uses approximately 200-400 bytes of memory
2. **Neighbor Lists**: Reduce `max_neighbors` if memory is limited
3. **Batch Processing**: Process agents in batches for very large simulations

### Computational Complexity

- **Agent-Agent Interactions**: O(N × max_neighbors) per step
- **Spatial Partitioning**: O(N log N) for neighbor finding
- **ORCA Computation**: O(max_neighbors) per agent per step

### Scaling Guidelines

| Agents | Recommended max_neighbors | Expected Performance |
|--------|---------------------------|---------------------|
| 1-50   | 10-15                    | Real-time capable |
| 50-200 | 8-12                     | Near real-time |
| 200-500| 5-8                      | Batch processing |
| 500+   | 3-5                      | Offline simulation |

## Debugging and Profiling

### Check OpenMP Status

```python
from rvo2_rl.util import openmp_max_threads, openmp_threads_used

def check_openmp_status():
    max_threads = openmp_max_threads()
    used_threads = openmp_threads_used()
    
    if max_threads == 1:
        print("❌ OpenMP not available")
        print("   Simulation will run single-threaded")
        print("   Consider installing a version with OpenMP support")
    else:
        print(f"✅ OpenMP available with {max_threads} threads")
        print(f"   Currently using {used_threads} threads")
        
        if used_threads < max_threads:
            print(f"   ⚠ Not using all available threads")
            print(f"   Set OMP_NUM_THREADS={max_threads} for best performance")

check_openmp_status()
```

### Performance Profiling

```python
import cProfile
import pstats
from rvo2_rl.rl import RVO2RLWrapper
from rvo2_rl.rvo2 import Vector2

def profile_simulation():
    """Profile a simulation to identify bottlenecks."""
    
    wrapper = RVO2RLWrapper(neighbor_dist=10.0, max_neighbors=5)
    
    # Add 100 agents
    for i in range(100):
        wrapper.add_agent(Vector2(i * 0.1, 0))
    
    goals = [Vector2(10, 0) for _ in range(100)]
    wrapper.set_goals(goals)
    wrapper.initialize()
    
    def simulation_loop():
        for _ in range(100):
            wrapper.set_preferred_velocities()
            wrapper.get_simulator().do_step()
    
    # Profile the simulation
    profiler = cProfile.Profile()
    profiler.enable()
    simulation_loop()
    profiler.disable()
    
    # Print results
    stats = pstats.Stats(profiler)
    stats.sort_stats('tottime')
    stats.print_stats(10)  # Top 10 functions

if __name__ == "__main__":
    profile_simulation()
```

This utilities module helps you optimize PYRVO2-RL performance for your specific hardware and use case.
