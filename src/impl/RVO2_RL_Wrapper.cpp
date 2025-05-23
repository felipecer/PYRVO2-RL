#include "RVO2_RL_Wrapper.h"
#include <random>
#include <cmath>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <sstream>
using std::string;
using std::vector;

namespace RL_EXTENSIONS
{
  void RVO2_RL_Wrapper::initialize()
  {
    // Check if the simulator is initialized
    if (!rvo_simulator_)
    {
      throw std::runtime_error("Simulator is not initialized.");
    }

    // Ensure there are agents in the simulator
    if (rvo_simulator_->getNumAgents() == 0)
    {
      throw std::runtime_error("No agents have been added to the simulator.");
    }

    // Ensure goals are set for all agents
    if (goal_vector_x_.size() != rvo_simulator_->getNumAgents() ||
        goal_vector_y_.size() != rvo_simulator_->getNumAgents())
    {
      throw std::runtime_error("Goals are not properly set for all agents.");
    }

    // If LIDAR is enabled, ensure the ray casting engine is initialized
    if (useLidar_ && !rayCastingEngine_)
    {
      throw std::runtime_error("LIDAR is enabled, but the RayCastingEngine is not initialized.");
    }

    // Set current positions and goals as initial values
    setCurrentPositionsAsInitialPositions();
    setCurrentGoalsAsInitialGoals();

    // Compute initial distances to goals
    computeDistancesToGoal();
  }

  // Return a mutable reference
  RVO::RVOSimulator &RVO2_RL_Wrapper::getSimulator()
  {
    return *rvo_simulator_;
  }

  // Return a const reference
  const RVO::RVOSimulator &RVO2_RL_Wrapper::getSimulator() const
  {
    return *rvo_simulator_;
  }

  RVO2_RL_Wrapper::RVO2_RL_Wrapper(
      float timeStep,
      float neighborDist,
      std::size_t maxNeighbors,
      float timeHorizon,
      float timeHorizonObst,
      float radius,
      float maxSpeed,
      const RVO::Vector2 &velocity,
      ObsMode mode,
      bool useObsMask,
      bool useLidar,
      std::size_t lidarCount,
      float lidarRange,
      std::size_t max_step_count)
      : rvo_simulator_{
            std::make_unique<RVO::RVOSimulator>(
                timeStep,
                neighborDist,
                maxNeighbors,
                timeHorizon,
                timeHorizonObst,
                radius,
                maxSpeed,
                velocity)},
        mode_{mode}, useObsMask_{useObsMask}, rayCastingEngine_{nullptr}, useLidar_{useLidar}, lidarCount_{lidarCount}, lidarRange_{lidarRange}, maxNeighbors_{maxNeighbors}
  {
    neighbor_dist_ = neighborDist;
    max_step_count_ = max_step_count;
    stepcount_ = 0;
    std::size_t n = rvo_simulator_->getNumAgents(); // típicamente 0 en este punto
    goal_vector_x_.assign(n, 0.0f);
    goal_vector_y_.assign(n, 0.0f);
    goal_initial_vector_x_.assign(n, 0.0f);
    goal_initial_vector_y_.assign(n, 0.0f);
    agent_pos_vector_x_.assign(n, 0.0f);
    agent_pos_vector_y_.assign(n, 0.0f);
    agent_initial_pos_vector_x_.assign(n, 0.0f);
    agent_initial_pos_vector_y_.assign(n, 0.0f);
    dist_to_goal_vector_x_.assign(n, 0.0f);
    dist_to_goal_vector_y_.assign(n, 0.0f);
    agent_behaviors_.assign(n, "");
    max_speed_ = maxSpeed;
    if (useLidar_)
    {
      rayCastingEngine_ = std::make_unique<RayCastingEngine>(lidarCount_, lidarRange_);
    }
  }
  RVO2_RL_Wrapper::~RVO2_RL_Wrapper() = default;

  size_t RVO2_RL_Wrapper::add_agent(const RVO::Vector2 &position)
  {
    // Add the agent to the RVO simulator
    size_t id = rvo_simulator_->addAgent(position);

    // Push the position of the new agent to the end of the position vectors
    agent_pos_vector_x_.push_back(position.x());
    agent_pos_vector_y_.push_back(position.y());

    return id;
  }

  size_t RVO2_RL_Wrapper::add_agent(const RVO::Vector2 &position,
                                    float neighborDist,
                                    size_t maxNeighbors,
                                    float timeHorizon,
                                    float timeHorizonObst,
                                    float radius,
                                    float maxSpeed,
                                    const RVO::Vector2 &velocity)
  {
    // Add the agent to the RVO simulator with all parameters
    size_t id = rvo_simulator_->addAgent(position,
                                         neighborDist,
                                         maxNeighbors,
                                         timeHorizon,
                                         timeHorizonObst,
                                         radius,
                                         maxSpeed,
                                         velocity);

    // Store the position of the new agent
    agent_pos_vector_x_.push_back(position.x());
    agent_pos_vector_y_.push_back(position.y());
    return id;
  }

  float RVO2_RL_Wrapper::getLidarRange() const
  {
    if (useLidar_ && rayCastingEngine_)
      return rayCastingEngine_->getRayLength();
    else
      return 0.0f;
  }
  void RVO2_RL_Wrapper::do_step()
  {
    rvo_simulator_->doStep();
    stepcount_++;
    computeAllAgentsPositions();
    computeDistancesToGoal();
  };

  std::size_t RVO2_RL_Wrapper::getStepCount()
  {
    return stepcount_;
  }

  void RVO2_RL_Wrapper::resetStepCount()
  {
    stepcount_ = 0;
  }

  // Returns a new vector<RVO::Vector2> combining the x/y arrays.
  std::vector<RVO::Vector2> RVO2_RL_Wrapper::getGoals() const
  {
    const size_t n = goal_vector_x_.size();
    std::vector<RVO::Vector2> out;
    out.reserve(n);

    for (size_t i = 0; i < n; ++i)
    {
      out.emplace_back(goal_vector_x_[i], goal_vector_y_[i]);
    }
    return out;
  }

  void RVO2_RL_Wrapper::setGoal(std::size_t agent_id, const RVO::Vector2 &goal)
  {
    if (agent_id >= goal_vector_x_.size())
    {
      throw std::out_of_range("setGoalSoA: bad agent_id");
    }
    goal_vector_x_[agent_id] = goal.x();
    goal_vector_y_[agent_id] = goal.y();
  }

  // Returns a single goal, throws if out‐of‐range.
  RVO::Vector2 RVO2_RL_Wrapper::getGoal(std::size_t agent_id) const
  {
    if (agent_id >= goal_vector_x_.size())
    {
      throw std::out_of_range("RVO2_RL_Wrapper::getGoal(): agent_id out of range");
    }
    return RVO::Vector2(goal_vector_x_[agent_id], goal_vector_y_[agent_id]);
  }

  void RVO2_RL_Wrapper::setGoals(const std::vector<RVO::Vector2> &goals)
  {
    const size_t n = goals.size();
    goal_vector_x_.resize(n);
    goal_vector_y_.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
      goal_vector_x_[i] = goals[i].x();
      goal_vector_y_[i] = goals[i].y();
    }
  }

  void RVO2_RL_Wrapper::setCurrentGoalsAsInitialGoals()
  {
    const size_t n = goal_vector_x_.size();
    goal_initial_vector_x_.resize(n);
    goal_initial_vector_y_.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
      goal_initial_vector_x_[i] = goal_vector_x_[i];
      goal_initial_vector_y_[i] = goal_vector_y_[i];
    }
  }

  void RVO2_RL_Wrapper::setCurrentPositionsAsInitialPositions()
  {
    const size_t n = agent_pos_vector_x_.size();
    agent_initial_pos_vector_x_.resize(n);
    agent_initial_pos_vector_y_.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
      agent_initial_pos_vector_x_[i] = agent_pos_vector_x_[i];
      agent_initial_pos_vector_y_[i] = agent_pos_vector_y_[i];
    }
  }

  void RVO2_RL_Wrapper::resetPositionAndGoalsToInit()
  {
    const size_t n = goal_initial_vector_x_.size();
    goal_vector_x_.resize(n);
    goal_vector_y_.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
      goal_vector_x_[i] = goal_initial_vector_x_[i];
      goal_vector_y_[i] = goal_initial_vector_y_[i];
    }
    const size_t m = agent_initial_pos_vector_x_.size();
    agent_pos_vector_x_.resize(m);
    agent_pos_vector_y_.resize(m);
    for (size_t i = 0; i < m; ++i)
    {
      agent_pos_vector_x_[i] = agent_initial_pos_vector_x_[i];
      agent_pos_vector_y_[i] = agent_initial_pos_vector_y_[i];
      rvo_simulator_->setAgentPosition(i, RVO::Vector2(agent_pos_vector_x_[i], agent_pos_vector_y_[i]));
    }
  }

  void RVO2_RL_Wrapper::setPreferredVelocity(int agent_id, RVO::Vector2 velocity)
  {
    // auto prefVel = rvo_simulator_->getAgentPrefVelocity(0);
    // auto newPrefVel = prefVel + velocity;
    // rvo_simulator_->setAgentPrefVelocity(agent_id, newPrefVel);
    rvo_simulator_->setAgentPrefVelocity(agent_id, velocity);
  }

  void RVO2_RL_Wrapper::setPreferredVelocities()
  {
    const int n = static_cast<int>(rvo_simulator_->getNumAgents());

    // 1) Update positions and directions
    computeAllAgentsPositions();
    computeDistancesToGoal();

    // 2) Generate perturbations
    std::vector<float> angles(n), dists(n);
    {
      std::mt19937 rng(std::random_device{}());
      std::uniform_real_distribution<float> angleDist(0.0f, 2.0f * M_PI);
      std::uniform_real_distribution<float> distDist(0.0f, 0.0001f);
      for (int i = 0; i < n; ++i)
      {
        angles[i] = angleDist(rng);
        dists[i] = distDist(rng);
      }
    }

    std::vector<float> vx(n), vy(n);

    // Step 1: SIMD calculation of preferred velocities with speed scaling
#pragma omp simd
    for (int i = 0; i < n; ++i)
    {
      float angle = angles[i];
      float dist = dists[i];
      float px = std::cos(angle);
      float py = std::sin(angle);

      // Get direction to goal
      float dx = dist_to_goal_vector_x_[i];
      float dy = dist_to_goal_vector_y_[i];

      // Normalize direction vector if non-zero
      float len = std::sqrt(dx * dx + dy * dy);
      if (len > 1e-6f)
      {
        dx /= len;
        dy /= len;
      }

      // Scale by agent's max speed and add small random perturbation
      float maxSpeed = rvo_simulator_->getAgentMaxSpeed(i);
      vx[i] = dx * maxSpeed + px * dist;
      vy[i] = dy * maxSpeed + py * dist;
    }

    // Step 2: Apply results (sequential or parallel)
#pragma omp parallel for
    for (int i = 0; i < n; ++i)
    {
      rvo_simulator_->setAgentPrefVelocity(i, RVO::Vector2(vx[i], vy[i]));
    }
  }

  void RVO2_RL_Wrapper::computeDistancesToGoal()
  {
    const int n = static_cast<int>(rvo_simulator_->getNumAgents());
    dist_to_goal_vector_x_.resize(n);
    dist_to_goal_vector_y_.resize(n);

#pragma omp simd
    for (int i = 0; i < n; ++i)
    {
      float dx = goal_vector_x_[i] - agent_pos_vector_x_[i];
      float dy = goal_vector_y_[i] - agent_pos_vector_y_[i];

      dist_to_goal_vector_x_[i] = dx;
      dist_to_goal_vector_y_[i] = dy;
    }
  }

  void RVO2_RL_Wrapper::computeDistanceToGoal(int agent_id)
  {
    float dx = goal_vector_x_[agent_id] - agent_pos_vector_x_[agent_id];
    float dy = goal_vector_y_[agent_id] - agent_pos_vector_y_[agent_id];
    dist_to_goal_vector_x_[agent_id] = dx;
    dist_to_goal_vector_y_[agent_id] = dy;
  }

  void RVO2_RL_Wrapper::computeAllAgentsPositions()
  {
    const int n = static_cast<int>(rvo_simulator_->getNumAgents());
    agent_pos_vector_x_.resize(n);
    agent_pos_vector_y_.resize(n);

    for (int i = 0; i < n; ++i)
    {
      auto p = rvo_simulator_->getAgentPosition(i);
      agent_pos_vector_x_[i] = p.x();
      agent_pos_vector_y_[i] = p.y();
    }
  }

  void RVO2_RL_Wrapper::getRayCast(int agent_id,
                                   std::vector<float> &out_x,
                                   std::vector<float> &out_y) const
  {
    /* ---------- 0) pre‑flight checks ---------------------------------- */
    if (!rayCastingEngine_)
      throw std::runtime_error("RayCastingEngine not initialised.");

    if (agent_id < 0 || static_cast<std::size_t>(agent_id) >= rvo_simulator_->agents_.size())
      throw std::out_of_range("agent_id out of range");

    const RVO::Vector2 origin = rvo_simulator_->agents_[agent_id]->position_;

    /* ---------- 1) collect ONLY the agent’s obstacle neighbours ------- */
    std::vector<std::pair<RVO::Vector2, RVO::Vector2>> segments;
    const std::size_t nObs = rvo_simulator_->getAgentNumObstacleNeighbors(agent_id);
    segments.reserve(nObs);

    for (std::size_t n = 0; n < nObs; ++n)
    {
      size_t v0Idx = rvo_simulator_->getAgentObstacleNeighbor(agent_id, n); // first vertex
      size_t v1Idx = rvo_simulator_->getNextObstacleVertexNo(v0Idx);        // next in polygon

      const RVO::Vector2 &v0 = rvo_simulator_->getObstacleVertex(v0Idx);
      const RVO::Vector2 &v1 = rvo_simulator_->getObstacleVertex(v1Idx);

      segments.emplace_back(v0, v1);
    }

    /* ---------- 2) intersect ray fan with these segments -------------- */
    std::vector<float> rHit;
    std::vector<int> segIdx;
    rayCastingEngine_->computeIntersections(origin, segments, rHit, segIdx);

    /* ---------- 3) convert (origin, dir, r) -> hit coordinates -------- */
    const auto &rays = rayCastingEngine_->getRays();
    const std::size_t N = rays.size();
    out_x.resize(N);
    out_y.resize(N);

    // const float NaN = std::numeric_limits<float>::quiet_NaN();

    for (std::size_t i = 0; i < N; ++i)
    {
      if (segIdx[i] == -1)
      { // miss
        out_x[i] = -9999;
        out_y[i] = -9999;
      }
      else
      { // hit
        RVO::Vector2 hit = origin + rays[i] * rHit[i];
        out_x[i] = hit.x();
        out_y[i] = hit.y();
      }
    }
  }

  void RVO2_RL_Wrapper::getRayCastingProcessed(int agent_id,
                                               std::vector<float> &outDistances,
                                               std::vector<uint8_t> &outMask) const
  {
    if (!rayCastingEngine_)
      throw std::runtime_error("RayCastingEngine not initialised.");

    if (agent_id < 0 || static_cast<std::size_t>(agent_id) >= rvo_simulator_->agents_.size())
      throw std::out_of_range("agent_id out of range");

    const RVO::Vector2 origin = rvo_simulator_->agents_[agent_id]->position_;

    // 1. Collect only obstacle neighbors
    std::vector<std::pair<RVO::Vector2, RVO::Vector2>> segments;
    const std::size_t nObs = rvo_simulator_->getAgentNumObstacleNeighbors(agent_id);
    segments.reserve(nObs);

    for (std::size_t n = 0; n < nObs; ++n)
    {
      size_t v0Idx = rvo_simulator_->getAgentObstacleNeighbor(agent_id, n);
      size_t v1Idx = rvo_simulator_->getNextObstacleVertexNo(v0Idx);
      segments.emplace_back(rvo_simulator_->getObstacleVertex(v0Idx), rvo_simulator_->getObstacleVertex(v1Idx));
    }

    // 2. Raycast
    std::vector<float> rHit;
    std::vector<int> segIdx;
    rayCastingEngine_->computeIntersections(origin, segments, rHit, segIdx);

    // 3. Normalize & mask
    const std::size_t N = rHit.size();
    outDistances.resize(N);
    outMask.resize(N);

    const float max_range = 1.0f; // same as ray length in `initRays`

    for (std::size_t i = 0; i < N; ++i)
    {
      if (segIdx[i] == -1 || std::isinf(rHit[i]) || rHit[i] > max_range)
      {
        outDistances[i] = 1.0f; // max = "no hit"
        outMask[i] = 0;
      }
      else
      {
        outDistances[i] = rHit[i]; // already ∈ [0, 1]
        outMask[i] = 1;
      }
    }
  }

  std::vector<BatchAgentData> RVO2_RL_Wrapper::collectAgentsBatchData() const
  {
    const std::size_t n = rvo_simulator_->agents_.size();
    std::vector<BatchAgentData> out;
    out.reserve(n);

    for (std::size_t i = 0; i < n; ++i)
    {
      const RVO::Agent *a = rvo_simulator_->agents_[i];
      BatchAgentData d;
      d.id = static_cast<int>(i);
      d.px = a->position_.x();
      d.py = a->position_.y();
      d.vx = a->velocity_.x();
      d.vy = a->velocity_.y();
      d.pvx = a->prefVelocity_.x();
      d.pvy = a->prefVelocity_.y();
      d.dist_goal = std::hypot(dist_to_goal_vector_x_[i], dist_to_goal_vector_y_[i]);

      out.push_back(d);
    }
    return out; // RVO; no extra copies in C++20 move‑elision
  }

  pybind11::array_t<float> RVO2_RL_Wrapper::getNeighborsObsPolar(int agent_id) const
  {
    auto curPosMainAgent = rvo_simulator_->getAgentPosition(agent_id);
    size_t maxNeighbors = rvo_simulator_->getAgentMaxNeighbors(agent_id);
    size_t n = rvo_simulator_->getAgentNumAgentNeighbors(agent_id);
    float max_dist_neighbors = neighbor_dist_;
    // pybind11::array_t<float> arr({(pybind11::ssize_t)maxNeighbors, 6});
    pybind11::array_t<float> arr({static_cast<pybind11::ssize_t>(maxNeighbors), static_cast<pybind11::ssize_t>(4)});
    auto buf = arr.mutable_unchecked<2>();
#pragma omp simd
    for (size_t i = 0; i < n; ++i)
    {
      size_t nbrId = rvo_simulator_->getAgentAgentNeighbor(agent_id, i);
      auto pos = rvo_simulator_->getAgentPosition(nbrId);
      auto vel = rvo_simulator_->getAgentVelocity(nbrId);
      float vx = vel.x(), vy = vel.y();
      float vel_mag = std::sqrt(vx * vx + vy * vy);
      float vel_angle = (vx == 0.0f && vy == 0.0f) ? 0.0f : std::atan2(vy, vx);
      buf(i, 0) = (pos.x() - curPosMainAgent.x())/max_dist_neighbors;
      buf(i, 1) = (pos.y() - curPosMainAgent.y())/max_dist_neighbors;
      buf(i, 2) = vel_mag;
      buf(i, 3) = vel_angle;
    }
#pragma omp simd
    for (size_t i = n; i < maxNeighbors; ++i)
    {
      buf(i, 0) = -2.0f; // pos_x invalid
      buf(i, 1) = -2.0f; // pos_y invalid
      buf(i, 2) = 0.0f;   // vel_mag
      buf(i, 3) = 0.0f;   // vel_angle
    }
    return arr;
  }

  pybind11::array_t<float> RVO2_RL_Wrapper::getNeighborsObsCartesian(int agent_id) const
  {
    auto curPosMainAgent = rvo_simulator_->getAgentPosition(agent_id);
    // Get maximum number of neighbors and the current active neighbor count
    size_t maxNeighbors = rvo_simulator_->getAgentMaxNeighbors(agent_id);
    size_t n = rvo_simulator_->getAgentNumAgentNeighbors(agent_id);
    float max_dist_neighbors = neighbor_dist_;
    // Create a NumPy array of shape (maxNeighbors, 4)
    // Columns: pos_x, pos_y, vel_x, vel_y, pref_vel_x, pref_vel_y
    pybind11::array_t<float> arr({static_cast<pybind11::ssize_t>(maxNeighbors), static_cast<pybind11::ssize_t>(4)});
    // Access the buffer without bounds checking for maximum performance
    auto buf = arr.mutable_unchecked<2>();

    // Fill in actual neighbor data for the first n rows
#pragma omp simd
    for (size_t i = 0; i < n; ++i)
    {
      // Retrieve neighbor ID and properties via the public API
      size_t nbrId = rvo_simulator_->getAgentAgentNeighbor(agent_id, i);
      auto pos = rvo_simulator_->getAgentPosition(nbrId);
      auto vel = rvo_simulator_->getAgentVelocity(nbrId);

      // Write position, velocity, and preferred velocity data
      buf(i, 0) = (pos.x() - curPosMainAgent.x())/max_dist_neighbors; // x-coordinate
      buf(i, 1) = (pos.y() - curPosMainAgent.y())/max_dist_neighbors; // y-coordinate
      buf(i, 2) = vel.x();                       // velocity x-component
      buf(i, 3) = vel.y();                       // velocity y-component
    }
#pragma omp simd
    // Pad remaining rows with an invalid position and zero velocities
    for (size_t i = n; i < maxNeighbors; ++i)
    {
      buf(i, 0) = -2.0f; // invalid pos_x
      buf(i, 1) = -2.0f; // invalid pos_y
      buf(i, 2) = 0.0f;   // vel_x
      buf(i, 3) = 0.0f;   // vel_y
    }

    // Return the NumPy array to Python
    return arr;
  }

  pybind11::array_t<float> RVO2_RL_Wrapper::getNeighborsObsPolarWithMask(int agent_id) const
  {
    auto curPosMainAgent = rvo_simulator_->getAgentPosition(agent_id);
    // Maximum neighbors and active neighbor count
    size_t maxNeighbors = rvo_simulator_->getAgentMaxNeighbors(agent_id);
    size_t n = rvo_simulator_->getAgentNumAgentNeighbors(agent_id);
    float max_dist_neighbors = neighbor_dist_;
    // Create a single NumPy array with shape (maxNeighbors, 7)
    // Columns: pos_x, pos_y, vel_mag, vel_angle, pref_vel_mag, pref_vel_angle, mask
    pybind11::array_t<float> arr({static_cast<pybind11::ssize_t>(maxNeighbors), static_cast<pybind11::ssize_t>(5)});
    auto buf = arr.mutable_unchecked<2>();

// Fill data and mask for actual neighbors
#pragma omp simd
    for (size_t i = 0; i < n; ++i)
    {
      size_t nbr = rvo_simulator_->getAgentAgentNeighbor(agent_id, i);
      auto pos = rvo_simulator_->getAgentPosition(nbr);
      auto vel = rvo_simulator_->getAgentVelocity(nbr);

      // Compute magnitude and angle of velocity
      float vx = vel.x(), vy = vel.y();
      float mag = std::sqrt(vx * vx + vy * vy);
      float ang = (vx == 0.0f && vy == 0.0f) ? 0.0f : std::atan2(vy, vx);

      // Write into buffer
      buf(i, 0) = (pos.x() - curPosMainAgent.x())/max_dist_neighbors;;
      buf(i, 1) = (pos.y() - curPosMainAgent.y())/max_dist_neighbors;;
      buf(i, 2) = mag;
      buf(i, 3) = ang;
      buf(i, 4) = 1.0f; // valid neighbor mask
    }

// Pad remaining entries with invalid/zero values and mask=0
#pragma omp simd
    for (size_t i = n; i < maxNeighbors; ++i)
    {
      buf(i, 0) = -2.0f; // invalid position x
      buf(i, 1) = -2.0f; // invalid position y
      buf(i, 2) = 0.0f;   // zero velocity magnitude
      buf(i, 3) = 0.0f;   // zero velocity angle
      buf(i, 4) = 0.0f;   // invalid neighbor mask
    }
    return arr;
  }

  // Generate a single NumPy array containing Cartesian neighbor observations plus a mask
  // for the specified agent. Each row has 7 columns:
  // [pos_x, pos_y, vel_x, vel_y, pref_vel_x, pref_vel_y, valid_mask]
  pybind11::array_t<float> RVO2_RL_Wrapper::getNeighborsObsCartesianWithMask(int agent_id) const
  {
    auto curPosMainAgent = rvo_simulator_->getAgentPosition(agent_id);
    // Retrieve maximum neighbors and active neighbor count
    size_t maxNeighbors = rvo_simulator_->getAgentMaxNeighbors(agent_id);
    size_t n = rvo_simulator_->getAgentNumAgentNeighbors(agent_id);
    float max_dist_neighbors = neighbor_dist_;
    // Create a NumPy array of shape (maxNeighbors, 7)
    // Last column is mask: 1.0 for valid neighbor, 0.0 for padding
    pybind11::array_t<float> arr({static_cast<pybind11::ssize_t>(maxNeighbors), static_cast<pybind11::ssize_t>(5)});
    auto buf = arr.mutable_unchecked<2>(); // fast unchecked access

// Populate actual neighbor entries
#pragma omp simd
    for (size_t i = 0; i < n; ++i)
    {
      // Get neighbor ID and properties via public API
      size_t nbrId = rvo_simulator_->getAgentAgentNeighbor(agent_id, i);
      auto pos = rvo_simulator_->getAgentPosition(nbrId);
      auto vel = rvo_simulator_->getAgentVelocity(nbrId);

      // Fill position and velocity components
      buf(i, 0) = (pos.x() - curPosMainAgent.x())/max_dist_neighbors; // x-coordinate
      buf(i, 1) = (pos.y() - curPosMainAgent.y())/max_dist_neighbors; // y-coordinate
      buf(i, 2) = vel.x();                       // velocity x-component
      buf(i, 3) = vel.y();                       // velocity y-component
      buf(i, 4) = 1.0f;
    }

// Pad remaining rows with invalid markers and zero motion
#pragma omp simd
    for (size_t i = n; i < maxNeighbors; ++i)
    {
      buf(i, 0) = -2.0f; // invalid pos_x
      buf(i, 1) = -2.0f; // invalid pos_y
      buf(i, 2) = 0.0f;   // vel_x
      buf(i, 3) = 0.0f;   // vel_y
      buf(i, 4) = 0.0f;   // mask invalid
    }

    // Return the combined data+mask array
    return arr;
  }

  pybind11::array_t<float> RVO2_RL_Wrapper::get_neighbors(int agent_id) const
  {
    if (mode_ == ObsMode::Cartesian)
    {
      return useObsMask_
                 ? getNeighborsObsCartesianWithMask(agent_id)
                 : getNeighborsObsCartesian(agent_id);
    }
    else
    {
      return useObsMask_
                 ? getNeighborsObsPolarWithMask(agent_id)
                 : getNeighborsObsPolar(agent_id);
    }
  }

  pybind11::array_t<float> RVO2_RL_Wrapper::get_lidar(int agent_id) const
  {
    if (!useLidar_)
    {
      throw std::runtime_error("LIDAR disabled on this instance");
    }

    // 1) get normalized ranges + mask
    std::vector<float> distances;
    std::vector<uint8_t> mask;
    getRayCastingProcessed(agent_id, distances, mask);

    const auto &rays = rayCastingEngine_->getRays(); // vector<RVO::Vector2>
    const ssize_t N = static_cast<pybind11::ssize_t>(distances.size());

    // 2) decide how many columns: angle + range [+ mask]
    const ssize_t cols = useObsMask_ ? 3 : 2;

    // 3) allocate output array (N x cols)
    // pybind11::array_t<float> arr({N, cols});
    pybind11::array_t<float> arr({N, cols});
    auto buf = arr.mutable_unchecked<2>();

    // 4) fill it in one pass
    for (ssize_t i = 0; i < N; ++i)
    {
      // compute angle from ray direction
      const auto &d = rays[i];
      float angle = std::atan2(d.y(), d.x());

      buf(i, 0) = angle;        // θ in radians, relative to +X axis
      buf(i, 1) = distances[i]; // normalized range ∈ [0,1]

      if (useObsMask_)
      {
        buf(i, 2) = float(mask[i]); // 1.0 = hit, 0.0 = miss
      }
      else
      {
        buf(i, 1) = distances[i] * 3;
      }
    }

    return arr;
  }

  pybind11::dict RVO2_RL_Wrapper::get_observation_bounds() const
  {
    using ssize = pybind11::ssize_t;

    std::vector<float> low, high;
    std::vector<std::string> info;

    // ─────────────────────────────────────────────────────────────────────
    // 1) step ∈ [0,1]
    // ─────────────────────────────────────────────────────────────────────
    std::size_t max_step_count = max_step_count_;
    low.push_back(0.0f);
    high.push_back((float)max_step_count);
    // info.push_back(std::to_string(max_step_count) + "]");
    info.push_back("[0] step ∈ [0," + std::to_string(max_step_count) + "]");

    // ─────────────────────────────────────────────────────────────────────
    // 2) dist_to_goal_x,y ∈ [-1,1]
    // ─────────────────────────────────────────────────────────────────────
    low.insert(low.end(), {-1.0f, -1.0f});
    high.insert(high.end(), {1.0f, 1.0f});
    info.push_back("[1:3] dist_to_goal_x,y ∈ [-1,1]");

    // ─────────────────────────────────────────────────────────────────────
    // 3) optional LIDAR block: [angle, range (,(mask))] * lidarCount_
    // ─────────────────────────────────────────────────────────────────────
    size_t idx = low.size();
    if (useLidar_)
    {
      const size_t per_ray = useObsMask_ ? 3 : 2;
      const size_t total = lidarCount_ * per_ray;

      // Reserve space (optional, but avoids reallocations)
      low.reserve(idx + total);
      high.reserve(idx + total);

      for (size_t i = 0; i < lidarCount_; ++i)
      {
        // a) angle ∈ [–π,π)
        low.push_back(-static_cast<float>(M_PI));
        high.push_back(static_cast<float>(M_PI));

        // b) normalized range ∈ [0,1]
        low.push_back(0.0f);
        high.push_back(1.0f);

        // c) optionally mask ∈ {0,1}
        if (useObsMask_)
        {
          low.push_back(0.0f);
          high.push_back(1.0f);
        }
      }

      // Build the human‐readable info line
      std::ostringstream oss;
      oss << "[" << idx << ":" << (idx + total) << ") lidar ";
      oss << (useObsMask_ ? "(angle,range,mask) * " : "(angle,range) * ");
      oss << lidarCount_;
      info.push_back(oss.str());

      // Advance our write‐cursor
      idx += total;
    }

    // ─────────────────────────────────────────────────────────────────────
    // 4) neighbor block: [features + optional mask] * maxNeighbors_
    //    Cartesian fields:    pos_x,  pos_y,  vel_x,  vel_y
    //    Polar    fields:    pos_x,  pos_y,  vel_mag,vel_ang
    // ─────────────────────────────────────────────────────────────────────
    const size_t base = low.size();
    const size_t feat_nbr = 4 + (useObsMask_ ? 1 : 0); // per‐neighbor column count
    // Pre‑reserve to avoid reallocations
    low.reserve(base + maxNeighbors_ * feat_nbr);
    high.reserve(base + maxNeighbors_ * feat_nbr);
    float max_speed = max_speed_;
    
    for (size_t i = 0; i < maxNeighbors_; ++i)
    {
      // 1) pos_x
      low.push_back(-2.0f);
      high.push_back(2.0f);
      // 2) pos_y
      low.push_back(-2.0f);
      high.push_back(2.0f);

      if (mode_ == ObsMode::Cartesian)
      {
        // 3) vel_x
        low.push_back(-max_speed);
        high.push_back(max_speed);
        // 4) vel_y
        low.push_back(-max_speed);
        high.push_back(max_speed);
      }
      else
      {
        // 3) vel_mag ∈ [0,1]
        low.push_back(0.0f);
        high.push_back(max_speed);
        // 4) vel_ang ∈ [−π,π)
        low.push_back(-static_cast<float>(M_PI));
        high.push_back(static_cast<float>(M_PI));
      }

      // 7) optional mask ∈ {0,1}
      if (useObsMask_)
      {
        low.push_back(0.0f);
        high.push_back(1.0f);
      }
    }

    // human‐readable slice annotation
    {
      std::ostringstream oss;
      oss << "[" << base << ":" << (base + maxNeighbors_ * feat_nbr)
          << ") neighbors ×" << maxNeighbors_
          << " ("
          << (mode_ == ObsMode::Cartesian
                  ? "pos_x,pos_y,vel_x,vel_y"
                  : "pos_x,pos_y,vel_mag,vel_ang")
          << (useObsMask_ ? ",mask)" : ")");
      info.push_back(oss.str());
    }

    // ─────────────────────────────────────────────────────────────────────
    // 6) pack into NumPy arrays and return dict
    // ─────────────────────────────────────────────────────────────────────
    auto low_arr = pybind11::array_t<float>(low.size(), low.data());
    auto high_arr = pybind11::array_t<float>(high.size(), high.data());

    pybind11::dict d;
    d["mode"] = (mode_ == ObsMode::Cartesian ? "cartesian" : "polar");
    d["low"] = low_arr;
    d["high"] = high_arr;
    d["info"] = info; // vector<string> → Python list[str]
    return d;
  }

  float RVO2_RL_Wrapper::getDistanceToGoal(size_t agent_id, bool normalized) const
  {
    if (agent_id >= dist_to_goal_vector_x_.size())
    {
      throw std::out_of_range("getDistanceToGoal: agent_id out of range");
    }
    float dx = dist_to_goal_vector_x_[agent_id];
    float dy = dist_to_goal_vector_y_[agent_id];
    float distance = std::sqrt(dx * dx + dy * dy);
    // pybind11::print("get_distance_to_goal: ", distance);
    if (normalized)
    {
      float og_dx = goal_initial_vector_x_[agent_id] - agent_initial_pos_vector_x_[agent_id];
      float og_dy = goal_initial_vector_y_[agent_id] - agent_initial_pos_vector_y_[agent_id];
      float og_distance = std::sqrt(og_dx * og_dx + og_dy * og_dy);
      // pybind11::print("og_distance: ", og_distance);
      if (og_distance > 0.0f)
      {
        distance /= og_distance;
      }
    }
    return distance;
  }

  std::vector<float> RVO2_RL_Wrapper::getAllDistancesToGoals(bool normalized) const
  {
    std::vector<float> distances;
    distances.reserve(dist_to_goal_vector_x_.size());

    for (size_t i = 0; i < dist_to_goal_vector_x_.size(); ++i)
    {
      float dx = dist_to_goal_vector_x_[i];
      float dy = dist_to_goal_vector_y_[i];
      float distance = std::sqrt(dx * dx + dy * dy);

      if (normalized)
      {
        float og_dx = goal_initial_vector_x_[i] - agent_initial_pos_vector_x_[i];
        float og_dy = goal_initial_vector_y_[i] - agent_initial_pos_vector_y_[i];
        float og_distance = std::sqrt(og_dx * og_dx + og_dy * og_dy);
        if (og_distance > 0.0f)
        {
          distance /= og_distance;
        }
      }

      distances.push_back(distance);
    }

    return distances;
  }
} // namespace RL_EXTENSIONS
