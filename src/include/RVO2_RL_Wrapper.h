#ifndef RL_EXTENSIONS_RL_WRAPPER
#define RL_EXTENSIONS_RL_WRAPPER
#include <vector>
#include "src/rvo2/include/Vector2.h"
#include <memory>
#include "RayCastingEngine.h"
#include "src/include/structs.h"
#include "src/rvo2/include/Agent.h"
#include "src/rvo2/include/RVOSimulator.h"
#include <pybind11/numpy.h>

namespace RL_EXTENSIONS
{
  enum class ObsMode
  {
    Cartesian = 0,
    Polar = 1
  };

  class RVO2_RL_Wrapper
  {
  private:
    float neighbor_dist_ = 5.0f;
    float max_speed_ = 2.0f;
    std::size_t stepcount_ = 0;
    std::size_t max_step_count_ = 256;
    std::unique_ptr<RayCastingEngine> rayCastingEngine_;
    std::vector<float> goal_initial_vector_x_; ///< SoA: x‐coords of goals
    std::vector<float> goal_initial_vector_y_;
    std::vector<float> goal_vector_x_; ///< SoA: x‐coords of goals
    std::vector<float> goal_vector_y_;
    std::vector<float> agent_pos_vector_x_;
    std::vector<float> agent_pos_vector_y_;
    std::vector<float> agent_initial_pos_vector_x_;
    std::vector<float> agent_initial_pos_vector_y_;
    std::vector<float> dist_to_goal_vector_x_;
    std::vector<float> dist_to_goal_vector_y_;
    std::size_t maxNeighbors_;
    bool useLidar_;
    std::size_t lidarCount_;
    float lidarRange_;
    std::unique_ptr<RVO::RVOSimulator> rvo_simulator_;
    ObsMode mode_;
    bool useObsMask_;
    std::vector<std::string> agent_behaviors_; // Stores behaviors for each agent
    friend class RVO::Agent;
    friend class RVO::KdTree;
    friend class RVO::Obstacle;
    pybind11::array_t<float> getNeighborsObsPolar(int agent_id) const;
    pybind11::array_t<float> getNeighborsObsCartesian(int agent_id) const;
    pybind11::array_t<float> getNeighborsObsPolarWithMask(int agent_id) const;
    pybind11::array_t<float> getNeighborsObsCartesianWithMask(int agent_id) const;

  public:
    RVO2_RL_Wrapper(
        float timeStep = 0.25f,
        float neighborDist = 15.0f,
        std::size_t maxNeighbors = 10,
        float timeHorizon = 5.0f,
        float timeHorizonObst = 5.0f,
        float radius = 0.5f,
        float maxSpeed = 2.0f,
        const RVO::Vector2 &velocity = RVO::Vector2(),
        ObsMode mode = ObsMode::Cartesian,
        bool useObsMask = false,
        bool useLidar = false,
        std::size_t lidarCount = 360,
        float lidarRange = 18.0f,
        std::size_t max_step_count = 256);
    void initialize();
    RVO::RVOSimulator &getSimulator();
    const RVO::RVOSimulator &getSimulator() const;
    pybind11::dict get_observation_bounds() const;
    std::vector<float> getAllDistancesToGoals(bool normalized) const;
    float getDistanceToGoal(size_t agent_id, bool normalized) const;
    ~RVO2_RL_Wrapper();
    pybind11::array_t<float> get_neighbors(int agent_id) const;
    pybind11::array_t<float> get_lidar(int agent_id) const;
    size_t add_agent(const RVO::Vector2 &position, float neighborDist,
                     size_t maxNeighbors, float timeHorizon,
                     float timeHorizonObst, float radius, float maxSpeed,
                     const RVO::Vector2 &velocity = RVO::Vector2());
    size_t add_agent(const RVO::Vector2 &position);

    float getLidarRange() const;
    void do_step();
    std::size_t getStepCount();
    void resetStepCount();

    std::vector<RVO::Vector2> getGoals() const;
    void setGoal(std::size_t agent_id, const RVO::Vector2 &goal);
    RVO::Vector2 getGoal(std::size_t agent_id) const;
    void setGoals(const std::vector<RVO::Vector2> &goals);
    void setCurrentGoalsAsInitialGoals();
    void setCurrentPositionsAsInitialPositions();
    void resetPositionAndGoalsToInit();
    void setPreferredVelocity(int agent_id, RVO::Vector2 velocity);
    void setPreferredVelocities();
    void computeDistancesToGoal();
    void computeDistanceToGoal(int agent_id);
    void computeAllAgentsPositions();

    void getRayCast(int agent_id, std::vector<float> &out_x, std::vector<float> &out_y) const;
    void getRayCastingProcessed(int agent_id, std::vector<float> &outDistances, std::vector<uint8_t> &outMask) const;
    std::vector<BatchAgentData> collectAgentsBatchData() const;

    // Getters
    const std::vector<float> &getGoalsX() const { return goal_vector_x_; }
    const std::vector<float> &getGoalsY() const { return goal_vector_y_; }
    const std::vector<float> &getAgentsPosX() const { return agent_pos_vector_x_; }
    const std::vector<float> &getAgentsPosY() const { return agent_pos_vector_y_; }
    const std::vector<float> &getDistToGoalsX() const { return dist_to_goal_vector_x_; }
    const std::vector<float> &getDistToGoalsY() const { return dist_to_goal_vector_y_; }
    ObsMode getMode() const { return mode_; }
    bool isUsingObsMask() const { return useObsMask_; }
    bool isUsingLidar() const { return useLidar_; }

    // Getter for an agent's behavior
    std::string getAgentBehavior(size_t agent_id) const
    {
      if (agent_id >= agent_behaviors_.size())
      {
        throw std::out_of_range("Agent ID out of range");
      }
      return agent_behaviors_[agent_id];
    }

    // Setter for an agent's behavior
    void setAgentBehavior(size_t agent_id, const std::string &behavior)
    {
      if (agent_id >= agent_behaviors_.size())
      {
        // resize to hold agent_id + 1 entries, defaulting new strings to ""
        agent_behaviors_.resize(agent_id + 1);
      }
      agent_behaviors_[agent_id] = behavior;
    }

    // Initialize behaviors for all agents
    void initializeAgentBehaviors(size_t num_agents, const std::string &default_behavior = "")
    {
      agent_behaviors_.resize(num_agents, default_behavior);
    }
  };
} // namespace RL_EXTENSIONS

#endif