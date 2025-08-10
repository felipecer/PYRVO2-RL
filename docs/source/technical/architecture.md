# Architecture & Implementation

This document explains the internal architecture of PYRVO2-RL and how its components work together to provide high-performance multi-agent simulation with RL extensions.

## System Overview

PYRVO2-RL is built as a layered architecture that combines the proven RVO2 collision avoidance library with RL-optimized extensions for machine learning research.

```
┌─────────────────────────────────────────────────────────────┐
│                    Python Layer                            │
├─────────────────────────────────────────────────────────────┤
│  RVO2RLWrapper  │  Observation  │  Action     │  Environment │
│  (Main API)     │  Processing   │  Interface  │  Integration │
├─────────────────────────────────────────────────────────────┤
│                    C++ Binding Layer                       │
├─────────────────────────────────────────────────────────────┤
│  RVO2 Bindings  │  RL Bindings  │  OpenMP     │  Ray Casting │
│                 │               │  Bindings   │  Engine      │
├─────────────────────────────────────────────────────────────┤
│                    Core Implementation                     │
├─────────────────────────────────────────────────────────────┤
│  RVOSimulator   │  KdTree       │  Agent      │  Obstacle    │
│  (Main Engine)  │  (Spatial)    │  (Entity)   │  (Static)    │
└─────────────────────────────────────────────────────────────┘
```

## Core Components

### 1. RVOSimulator (Core Engine)

The heart of the simulation, responsible for agent coordination and collision avoidance.

**Key Responsibilities:**
- Agent lifecycle management (add, remove, update)
- Time stepping and simulation progression
- Obstacle processing and spatial indexing
- ORCA constraint computation

**Implementation Details:**
```cpp
class RVOSimulator {
private:
    std::vector<Agent*> agents_;
    std::vector<Obstacle*> obstacles_;
    KdTree* kdTree_;
    float timeStep_;
    float globalTime_;

public:
    void doStep();              // Main simulation step
    size_t addAgent(...);       // Add new agent
    void setAgentPrefVelocity(...); // Set preferred velocity
    Vector2 getAgentPosition(...);  // Get agent position
};
```

**Simulation Loop:**
1. **Compute New Velocities:** For each agent, calculate optimal velocity using ORCA
2. **Update Positions:** Move agents based on computed velocities
3. **Update Time:** Advance global simulation time
4. **Rebuild Spatial Index:** Update KdTree for efficient neighbor queries

### 2. Agent Management

Each agent maintains its state and participates in collision avoidance.

**Agent Properties:**
```cpp
class Agent {
private:
    Vector2 position_;          // Current position
    Vector2 velocity_;          // Current velocity
    Vector2 prefVelocity_;      // Preferred velocity (goal direction)
    float radius_;              // Agent radius
    float maxSpeed_;            // Maximum speed
    float neighborDist_;        // Neighbor detection distance
    size_t maxNeighbors_;       // Maximum neighbors to consider
    
    std::vector<Line> orcaLines_; // ORCA constraints
    std::vector<std::pair<float, const Agent*>> agentNeighbors_;
    std::vector<std::pair<float, const Obstacle*>> obstacleNeighbors_;
};
```

**ORCA Algorithm:**
The Optimal Reciprocal Collision Avoidance (ORCA) algorithm computes collision-free velocities:

1. **Neighbor Detection:** Find nearby agents and obstacles using KdTree
2. **Constraint Generation:** Create half-plane constraints (ORCA lines) for each neighbor
3. **Linear Programming:** Solve 2D linear program to find optimal velocity
4. **Fallback:** If no solution exists, find closest feasible velocity

### 3. Spatial Indexing (KdTree)

Efficient spatial data structure for fast neighbor queries.

**Structure:**
```cpp
class KdTree {
private:
    struct AgentTreeNode {
        size_t begin, end;      // Range of agents
        size_t left, right;     // Child nodes
        float minX, maxX, minY, maxY; // Bounding box
    };
    
    std::vector<AgentTreeNode> tree_;
    std::vector<Agent*> agents_;
};
```

**Performance Characteristics:**
- **Build Time:** O(n log n) where n is number of agents
- **Query Time:** O(log n + k) where k is number of neighbors found
- **Memory:** O(n) space complexity
- **Rebuilt:** Every simulation step for dynamic agents

### 4. Obstacle Processing

Static obstacles are preprocessed for efficient collision detection.

**Obstacle Representation:**
```cpp
class Obstacle {
private:
    Vector2 point_;           // Obstacle vertex
    Obstacle* next_;          // Next vertex (forms polygon)
    Obstacle* previous_;      // Previous vertex
    size_t id_;              // Obstacle ID
    bool isConvex_;          // Convexity flag
};
```

**Processing Steps:**
1. **Validation:** Ensure obstacles form valid polygons
2. **Convexity Detection:** Mark convex obstacles for optimization
3. **Spatial Indexing:** Build separate KdTree for obstacle queries
4. **Normal Computation:** Precompute outward normals for collision detection

## RL Extensions Layer

### 1. RVO2RLWrapper

The main Python interface that wraps RVO2 functionality with RL-specific features.

**Key Features:**
- **Observation Processing:** Multiple observation modes (polar, cartesian, raw)
- **Goal Management:** Automatic goal assignment and tracking
- **LiDAR Simulation:** Ray-casting for sensor simulation
- **Batch Operations:** Efficient batch processing for multiple agents

**Architecture:**
```python
class RVO2RLWrapper:
    def __init__(self, ...):
        self._sim = RVOSimulator(...)  # Core simulator
        self._ray_engine = RayCastingEngine(...)  # LiDAR simulation
        self._goals = []               # Goal management
        self._observation_cache = {}   # Performance optimization
```

### 2. Observation Processing

Flexible observation generation for different RL algorithms.

**Observation Modes:**

| Mode | Format | Use Case |
|------|---------|----------|
| `POLAR` | (distance, angle, velocity_radial, velocity_tangential, [mask]) | Rotation-invariant policies |
| `CARTESIAN` | (rel_x, rel_y, vel_x, vel_y, [mask]) | Simple CNN/MLP processing |
| `RAW` | (abs_x, abs_y, vel_x, vel_y, [mask]) | Global state information |

**Processing Pipeline:**
1. **Neighbor Query:** Get nearby agents using KdTree
2. **Coordinate Transform:** Convert to desired coordinate system
3. **Normalization:** Scale values to [0,1] or [-1,1] ranges
4. **Masking:** Add validity masks for variable-length observations
5. **Caching:** Store results to avoid recomputation

### 3. LiDAR Ray Casting

High-performance ray casting for sensor simulation.

**Implementation:**
```cpp
class RayCastingEngine {
private:
    struct Ray {
        Vector2 origin;
        Vector2 direction;
        float maxDistance;
    };
    
    std::vector<Obstacle*> obstacles_;
    float resolution_;        // Angular resolution
    size_t rayCount_;        // Number of rays
    
public:
    std::vector<float> castRays(Vector2 position, float range);
    bool rayIntersectsObstacle(const Ray& ray, const Obstacle* obs);
};
```

**Ray Casting Algorithm:**
1. **Ray Generation:** Create rays at uniform angular intervals
2. **Intersection Testing:** Test each ray against all obstacles
3. **Distance Calculation:** Find closest intersection per ray
4. **Result Processing:** Normalize distances and apply masks

**Performance Optimizations:**
- **Spatial Culling:** Skip obstacles outside ray range
- **Early Termination:** Stop at first intersection for opaque obstacles
- **SIMD Instructions:** Vectorized intersection calculations where possible

## OpenMP Parallelization

PYRVO2-RL uses OpenMP for multi-core parallelization of computationally intensive operations.

### Parallel Regions

**1. Agent Velocity Computation:**
```cpp
#pragma omp parallel for
for (size_t i = 0; i < agents_.size(); ++i) {
    agents_[i]->computeNeighbors(kdTree_);
    agents_[i]->computeNewVelocity();
}
```

**2. Position Updates:**
```cpp
#pragma omp parallel for
for (size_t i = 0; i < agents_.size(); ++i) {
    agents_[i]->update();
}
```

**3. KdTree Construction:**
```cpp
void KdTree::buildAgentTreeRecursive(size_t node) {
    #pragma omp task if (end - begin > threshold)
    {
        // Recursive tree building
        buildAgentTreeRecursive(leftChild);
        buildAgentTreeRecursive(rightChild);
    }
}
```

### Thread Safety Considerations

**Read-Only Data:**
- Agent positions and velocities (during neighbor computation)
- Obstacle geometry
- Simulation parameters

**Thread-Local Data:**
- ORCA constraints computation
- Neighbor lists
- Temporary calculation buffers

**Synchronization Points:**
- KdTree rebuilding (barrier)
- Global time advancement (single thread)
- Result aggregation (reduction)

## Memory Management

### Object Lifecycle

**Agent Management:**
```cpp
// Agent creation
size_t addAgent(const Vector2& position) {
    agents_.push_back(new Agent(position, ...));
    return agents_.size() - 1;
}

// Agent cleanup
~RVOSimulator() {
    for (Agent* agent : agents_) {
        delete agent;
    }
}
```

**Memory Pools:**
For high-performance scenarios, consider implementing memory pools:
```cpp
class AgentPool {
private:
    std::vector<Agent> pool_;
    std::vector<size_t> freeList_;
    
public:
    Agent* acquire();
    void release(Agent* agent);
};
```

### Cache Optimization

**Data Layout:**
- **Structure of Arrays (SoA):** Better for SIMD operations
- **Array of Structures (AoS):** Better for object-oriented access

**Example SoA Layout:**
```cpp
struct AgentData {
    std::vector<float> posX, posY;      // Positions
    std::vector<float> velX, velY;      // Velocities
    std::vector<float> radius;          // Radii
    std::vector<size_t> maxNeighbors;   // Parameters
};
```

## Integration Points

### Python Bindings

Using pybind11 for seamless Python-C++ integration:

```cpp
PYBIND11_MODULE(rvo2_rl, m) {
    py::class_<RVOSimulator>(m, "RVOSimulator")
        .def(py::init<>())
        .def("addAgent", &RVOSimulator::addAgent)
        .def("doStep", &RVOSimulator::doStep)
        .def("getAgentPosition", &RVOSimulator::getAgentPosition);
    
    py::class_<Vector2>(m, "Vector2")
        .def(py::init<float, float>())
        .def("x", &Vector2::x)
        .def("y", &Vector2::y);
}
```

### Gymnasium Integration

Standard interface for RL environments:

```python
class RVO2Environment(gym.Env):
    def __init__(self, num_agents=10):
        self.wrapper = RVO2RLWrapper(...)
        self.action_space = gym.spaces.Box(...)
        self.observation_space = gym.spaces.Box(...)
    
    def step(self, actions):
        # Apply actions to agents
        # Step simulation
        # Return observations, rewards, done, info
        
    def reset(self):
        # Reset simulation state
        # Return initial observations
```

## Performance Characteristics

### Scalability Analysis

**Time Complexity:**
- **N agents, M obstacles:** O(N log N + N*M) per step
- **Neighbor queries:** O(N log N) via KdTree
- **ORCA computation:** O(N*k) where k is max neighbors
- **Position updates:** O(N)

**Memory Usage:**
- **Base overhead:** ~400 bytes per agent
- **Neighbor storage:** O(N*k) dynamic allocation
- **KdTree:** O(N) additional space
- **Obstacles:** O(M) static allocation

**Threading Efficiency:**
- **Near-linear speedup** for N > number of cores
- **Load balancing** via OpenMP work-stealing
- **Cache efficiency** through spatial locality

### Optimization Guidelines

**For Maximum Performance:**
1. **Reduce neighbor search radius** when possible
2. **Limit maximum neighbors** per agent
3. **Use larger time steps** for less precision-critical applications
4. **Enable OpenMP** with appropriate thread count
5. **Minimize Python-C++ boundary crossings**

**Memory Optimization:**
1. **Preallocate agent capacity** if known
2. **Reuse obstacle geometry** across simulations
3. **Clear observation caches** periodically
4. **Use appropriate data types** (float vs double)

## Extension Points

### Custom Behaviors

**Subclassing Agents:**
```cpp
class CustomAgent : public Agent {
public:
    void computeNewVelocity() override {
        // Custom velocity computation
        Agent::computeNewVelocity(); // Call base implementation
        // Post-process velocity
    }
};
```

**Custom Observation Processing:**
```python
class CustomWrapper(RVO2RLWrapper):
    def get_observations(self, agent_ids=None):
        base_obs = super().get_observations(agent_ids)
        # Add custom features
        return enhanced_observations
```

### Plugin Architecture

**Future Extension Points:**
- **Custom distance metrics** for neighbor computation
- **Alternative collision avoidance algorithms** (RRT*, APF)
- **Dynamic obstacle predictors** for moving obstacles
- **Learning-based velocity selectors** instead of ORCA

This architecture provides a solid foundation for research while maintaining the flexibility to extend and customize behavior for specific applications.
