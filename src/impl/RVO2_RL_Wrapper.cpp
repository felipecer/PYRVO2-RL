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
      float lidarRange)
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
    if (useLidar_)
    {
      rayCastingEngine_ = std::make_unique<RayCastingEngine>(lidarCount_, lidarRange_);
    }
  }
  RVO2_RL_Wrapper::~RVO2_RL_Wrapper() = default;

  float RVO2_RL_Wrapper::getLidarRange() const
  {
    if (useLidar_ && rayCastingEngine_)
      return rayCastingEngine_->getRayLength();
    else
      return 0.0f;
  };

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
    } 
  }

  void RVO2_RL_Wrapper::setPreferredVelocities()
  {
    const int n = static_cast<int>(rvo_simulator_->getNumAgents());

    // 1) Actualizar posiciones y direcciones
    computeAllAgentsPositions();
    computeDistancesToGoal(); // ← llena dir_x_ y dir_y_

    // 2) Generar perturbaciones
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

// Paso 1: cálculo SIMD de velocidades preferidas
#pragma omp simd
    for (int i = 0; i < n; ++i)
    {
      float angle = angles[i];
      float dist = dists[i];
      float px = std::cos(angle);
      float py = std::sin(angle);
      vx[i] = dist_to_goal_vector_x_[i] + px * dist;
      vy[i] = dist_to_goal_vector_y_[i] + py * dist;
    }

// Paso 2: aplicar resultados (sec. o paralelo si quieres)
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
      float sq = dx * dx + dy * dy;
      // if (sq > 1.0f)
      // {
      //   float inv = 1.0f / std::sqrt(sq);
      //   dx *= inv;
      //   dy *= inv;
      // }
      dist_to_goal_vector_x_[i] = dx;
      dist_to_goal_vector_y_[i] = dy;
    }
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
    size_t maxNeighbors = rvo_simulator_->getAgentMaxNeighbors(agent_id);
    size_t n = rvo_simulator_->getAgentNumAgentNeighbors(agent_id);

    // pybind11::array_t<float> arr({(pybind11::ssize_t)maxNeighbors, 6});
    pybind11::array_t<float> arr({static_cast<pybind11::ssize_t>(maxNeighbors), static_cast<pybind11::ssize_t>(6)});
    auto buf = arr.mutable_unchecked<2>();
#pragma omp simd
    for (size_t i = 0; i < n; ++i)
    {
      size_t nbrId = rvo_simulator_->getAgentAgentNeighbor(agent_id, i);
      auto pos = rvo_simulator_->getAgentPosition(nbrId);
      auto vel = rvo_simulator_->getAgentVelocity(nbrId);
      auto pref = rvo_simulator_->getAgentPrefVelocity(nbrId);
      float vx = vel.x(), vy = vel.y();
      float vel_mag = std::sqrt(vx * vx + vy * vy);
      float vel_angle = (vx == 0.0f && vy == 0.0f) ? 0.0f : std::atan2(vy, vx);
      float pvx = pref.x(), pvy = pref.y();
      float pv_mag = std::sqrt(pvx * pvx + pvy * pvy);
      float pv_angle = (pvx == 0.0f && pvy == 0.0f) ? 0.0f : std::atan2(pvy, pvx);
      buf(i, 0) = pos.x();
      buf(i, 1) = pos.y();
      buf(i, 2) = vel_mag;
      buf(i, 3) = vel_angle;
      buf(i, 4) = pv_mag;
      buf(i, 5) = pv_angle;
    }
#pragma omp simd
    for (size_t i = n; i < maxNeighbors; ++i)
    {
      buf(i, 0) = -999.0f; // pos_x invalid
      buf(i, 1) = -999.0f; // pos_y invalid
      buf(i, 2) = 0.0f;    // vel_mag
      buf(i, 3) = 0.0f;    // vel_angle
      buf(i, 4) = 0.0f;    // pref_vel_mag
      buf(i, 5) = 0.0f;    // pref_vel_angle
    }
    return arr;
  }

  pybind11::array_t<float> RVO2_RL_Wrapper::getNeighborsObsCartesian(int agent_id) const
  {
    // Get maximum number of neighbors and the current active neighbor count
    size_t maxNeighbors = rvo_simulator_->getAgentMaxNeighbors(agent_id);
    size_t n = rvo_simulator_->getAgentNumAgentNeighbors(agent_id);

    // Create a NumPy array of shape (maxNeighbors, 6)
    // Columns: pos_x, pos_y, vel_x, vel_y, pref_vel_x, pref_vel_y
    // pybind11::array_t<float> arr({(pybind11::ssize_t)maxNeighbors, 6});
    pybind11::array_t<float> arr({static_cast<pybind11::ssize_t>(maxNeighbors), static_cast<pybind11::ssize_t>(6)});
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
      auto pref = rvo_simulator_->getAgentPrefVelocity(nbrId);

      // Write position, velocity, and preferred velocity data
      buf(i, 0) = pos.x();  // x-coordinate
      buf(i, 1) = pos.y();  // y-coordinate
      buf(i, 2) = vel.x();  // velocity x-component
      buf(i, 3) = vel.y();  // velocity y-component
      buf(i, 4) = pref.x(); // preferred velocity x-component
      buf(i, 5) = pref.y(); // preferred velocity y-component
    }
#pragma omp simd
    // Pad remaining rows with an invalid position and zero velocities
    for (size_t i = n; i < maxNeighbors; ++i)
    {
      buf(i, 0) = -999.0f; // invalid pos_x
      buf(i, 1) = -999.0f; // invalid pos_y
      buf(i, 2) = 0.0f;    // vel_x
      buf(i, 3) = 0.0f;    // vel_y
      buf(i, 4) = 0.0f;    // pref_vel_x
      buf(i, 5) = 0.0f;    // pref_vel_y
    }

    // Return the NumPy array to Python
    return arr;
  }

  pybind11::array_t<float> RVO2_RL_Wrapper::getNeighborsObsPolarWithMask(int agent_id) const
  {
    // Maximum neighbors and active neighbor count
    size_t maxNeighbors = rvo_simulator_->getAgentMaxNeighbors(agent_id);
    size_t n = rvo_simulator_->getAgentNumAgentNeighbors(agent_id);

    // Create a single NumPy array with shape (maxNeighbors, 7)
    // Columns: pos_x, pos_y, vel_mag, vel_angle, pref_vel_mag, pref_vel_angle, mask
    // pybind11::array_t<float> arr({(pybind11::ssize_t)maxNeighbors, 7});
    pybind11::array_t<float> arr({static_cast<pybind11::ssize_t>(maxNeighbors), static_cast<pybind11::ssize_t>(7)});
    auto buf = arr.mutable_unchecked<2>();

// Fill data and mask for actual neighbors
#pragma omp simd
    for (size_t i = 0; i < n; ++i)
    {
      size_t nbr = rvo_simulator_->getAgentAgentNeighbor(agent_id, i);
      auto pos = rvo_simulator_->getAgentPosition(nbr);
      auto vel = rvo_simulator_->getAgentVelocity(nbr);
      auto pref = rvo_simulator_->getAgentPrefVelocity(nbr);

      // Compute magnitude and angle of velocity
      float vx = vel.x(), vy = vel.y();
      float mag = std::sqrt(vx * vx + vy * vy);
      float ang = (vx == 0.0f && vy == 0.0f) ? 0.0f : std::atan2(vy, vx);

      // Compute magnitude and angle of preferred velocity
      float pvx = pref.x(), pvy = pref.y();
      float pmag = std::sqrt(pvx * pvx + pvy * pvy);
      float pang = (pvx == 0.0f && pvy == 0.0f) ? 0.0f : std::atan2(pvy, pvx);

      // Write into buffer
      buf(i, 0) = pos.x();
      buf(i, 1) = pos.y();
      buf(i, 2) = mag;
      buf(i, 3) = ang;
      buf(i, 4) = pmag;
      buf(i, 5) = pang;
      buf(i, 6) = 1.0f; // valid neighbor mask
    }

// Pad remaining entries with invalid/zero values and mask=0
#pragma omp simd
    for (size_t i = n; i < maxNeighbors; ++i)
    {
      buf(i, 0) = -999.0f; // invalid position x
      buf(i, 1) = -999.0f; // invalid position y
      buf(i, 2) = 0.0f;    // zero velocity magnitude
      buf(i, 3) = 0.0f;    // zero velocity angle
      buf(i, 4) = 0.0f;    // zero preferred vel magnitude
      buf(i, 5) = 0.0f;    // zero preferred vel angle
      buf(i, 6) = 0.0f;    // padding mask
    }

    return arr;
  }

  // Generate a single NumPy array containing Cartesian neighbor observations plus a mask
  // for the specified agent. Each row has 7 columns:
  // [pos_x, pos_y, vel_x, vel_y, pref_vel_x, pref_vel_y, valid_mask]
  pybind11::array_t<float> RVO2_RL_Wrapper::getNeighborsObsCartesianWithMask(int agent_id) const
  {
    // Retrieve maximum neighbors and active neighbor count
    size_t maxNeighbors = rvo_simulator_->getAgentMaxNeighbors(agent_id);
    size_t n = rvo_simulator_->getAgentNumAgentNeighbors(agent_id);

    // Create a NumPy array of shape (maxNeighbors, 7)
    // Last column is mask: 1.0 for valid neighbor, 0.0 for padding
    // pybind11::array_t<float> arr({(pybind11::ssize_t)maxNeighbors, 7});
    pybind11::array_t<float> arr({static_cast<pybind11::ssize_t>(maxNeighbors), static_cast<pybind11::ssize_t>(7)});
    auto buf = arr.mutable_unchecked<2>(); // fast unchecked access

// Populate actual neighbor entries
#pragma omp simd
    for (size_t i = 0; i < n; ++i)
    {
      // Get neighbor ID and properties via public API
      size_t nbrId = rvo_simulator_->getAgentAgentNeighbor(agent_id, i);
      auto pos = rvo_simulator_->getAgentPosition(nbrId);
      auto vel = rvo_simulator_->getAgentVelocity(nbrId);
      auto pref = rvo_simulator_->getAgentPrefVelocity(nbrId);

      // Fill position and velocity components
      buf(i, 0) = pos.x();  // x-coordinate
      buf(i, 1) = pos.y();  // y-coordinate
      buf(i, 2) = vel.x();  // velocity x-component
      buf(i, 3) = vel.y();  // velocity y-component
      buf(i, 4) = pref.x(); // preferred velocity x-component
      buf(i, 5) = pref.y(); // preferred velocity y-component

      // Mark as valid neighbor
      buf(i, 6) = 1.0f;
    }

// Pad remaining rows with invalid markers and zero motion
#pragma omp simd
    for (size_t i = n; i < maxNeighbors; ++i)
    {
      buf(i, 0) = -999.0f; // invalid pos_x
      buf(i, 1) = -999.0f; // invalid pos_y
      buf(i, 2) = 0.0f;    // vel_x
      buf(i, 3) = 0.0f;    // vel_y
      buf(i, 4) = 0.0f;    // pref_vel_x
      buf(i, 5) = 0.0f;    // pref_vel_y
      buf(i, 6) = 0.0f;    // mask indicating padding
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

  pybind11::dict RVO2_RL_Wrapper::get_observation_bounds() const {
    std::vector<float>       low, high;
    std::vector<std::string> info;

    // 1) step
    low .push_back(0.0f);    high.push_back(1.0f);
    info.push_back("[0] step ∈ [0,1]");

    // 2) agent position
    low .insert(low.end(),  { -1000.0f, -1000.0f });
    high.insert(high.end(), {  1000.0f,  1000.0f });
    info.push_back("[1:3] agent position x,y ∈ [-1000,1000]");

    // 3) optional LIDAR
    size_t idx = low.size();
    if (useLidar_) {
        // angles ∈ [-π,π)
        low .insert(low.end(),  lidarCount_, -M_PI);
        high.insert(high.end(), lidarCount_,  M_PI);
        {
            std::ostringstream oss;
            oss << "[" << idx << ":" << (idx + lidarCount_)
                << ") LIDAR angles ∈ [-π,π)";
            info.push_back(oss.str());
        }
        idx += lidarCount_;

        // normalized ranges ∈ [0,1]
        low .insert(low.end(),  lidarCount_, 0.0f);
        high.insert(high.end(), lidarCount_, 1.0f);
        {
            std::ostringstream oss;
            oss << "[" << idx << ":" << (idx + lidarCount_)
                << "] LIDAR normalized ranges ∈ [0,1]";
            info.push_back(oss.str());
        }
        idx += lidarCount_;

        if (useObsMask_) {
            // hit-mask ∈ {0,1}
            low .insert(low.end(),  lidarCount_, 0.0f);
            high.insert(high.end(), lidarCount_, 1.0f);
            std::ostringstream oss;
            oss << "[" << idx << ":" << (idx + lidarCount_)
                << "] LIDAR hit-mask ∈ {0,1}";
            info.push_back(oss.str());
            idx += lidarCount_;
        }
    }

    // 4) neighbor block (Cartesian vs Polar)
    size_t base = idx;
    for (size_t i = 0; i < maxNeighbors_; ++i) {
        // pos x,y
        low .push_back(-1000.0f); high.push_back(1000.0f);
        low .push_back(-1000.0f); high.push_back(1000.0f);
        // velocity/mag
        low .push_back(0.0f); high.push_back(1.0f);
        if (mode_ == ObsMode::Cartesian) {
            low .push_back(-1000.0f); high.push_back(1000.0f);
        } else {
            low .push_back(-M_PI);    high.push_back(M_PI);
        }
        // preferred velocity/mag
        low .push_back(0.0f); high.push_back(1.0f);
        if (mode_ == ObsMode::Cartesian) {
            low .push_back(-1000.0f); high.push_back(1000.0f);
        } else {
            low .push_back(-M_PI);    high.push_back(M_PI);
        }
    }
    {
        std::ostringstream oss;
        oss << "[" << base << ":" << (base + 6*maxNeighbors_)
            << "] neighbors (" << maxNeighbors_ << "×"
            << (mode_==ObsMode::Cartesian
                  ? "pos_x,pos_y,vel_x,vel_y,pref_x,pref_y"
                  : "pos_x,pos_y,vel_mag,vel_ang,pref_mag,pref_ang")
            << ")";
        info.push_back(oss.str());
    }
    idx = base + 6*maxNeighbors_;

    // 5) neighbor mask
    if (useObsMask_) {
        low .insert(low.end(),  maxNeighbors_, 0.0f);
        high.insert(high.end(), maxNeighbors_, 1.0f);
        std::ostringstream oss;
        oss << "[" << idx << ":" << (idx + maxNeighbors_)
            << "] neighbor hit-mask ∈ {0,1}";
        info.push_back(oss.str());
    }

    // wrap into NumPy arrays (no extra copy)
    auto low_arr  = pybind11::array_t<float>(low.size(),  low.data());
    auto high_arr = pybind11::array_t<float>(high.size(), high.data());

    // return dict with mode, arrays, and human-readable info
    pybind11::dict d;
    d["mode"]  = (mode_==ObsMode::Cartesian ? "cartesian" : "polar");
    d["low"]   = low_arr;
    d["high"]  = high_arr;
    d["info"]  = info;  // vector<string> → Python list[str]
    return d;
}  

  float RVO2_RL_Wrapper::getDistanceToGoal(size_t agent_id, bool normalized) const
  {
    if (agent_id >= dist_to_goal_vector_x_.size())
    {
      throw std::out_of_range("getDistanceToGoal: agent_id out of range");
    }

    float dx = dist_to_goal_vector_x_[agent_id];
    pybind11::print("dx: ", dx);
    float dy = dist_to_goal_vector_y_[agent_id];
    pybind11::print("dx: ", dy);
    float distance = std::sqrt(dx * dx + dy * dy);

    if (normalized)
    {
      
      float og_dx = goal_initial_vector_x_[agent_id] - agent_initial_pos_vector_x_[agent_id];
      pybind11::print("Hello 2");
      float og_dy = goal_initial_vector_y_[agent_id] - agent_initial_pos_vector_y_[agent_id];
      pybind11::print("Hello 3");
      float og_distance = std::sqrt(og_dx * og_dx + og_dy * og_dy);
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
