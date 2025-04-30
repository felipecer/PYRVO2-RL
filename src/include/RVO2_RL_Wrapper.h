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
    std::unique_ptr<RayCastingEngine> rayCastingEngine_;
    std::vector<float> goal_vector_x_; ///< SoA: x‐coords of goals
    std::vector<float> goal_vector_y_;
    std::vector<float> agent_pos_vector_x_;
    std::vector<float> agent_pos_vector_y_;
    std::vector<float> dist_to_goal_vector_x_;
    std::vector<float> dist_to_goal_vector_y_;
    std::size_t maxNeighbors_;
    bool useLidar_;
    std::size_t lidarCount_;
    float lidarRange_;
    std::unique_ptr<RVO::RVOSimulator> rvo_simulator_;
    ObsMode mode_;
    bool useObsMask_;
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
        float lidarRange = 18.0f);
    RVO::RVOSimulator &getSimulator();
    const RVO::RVOSimulator &getSimulator() const;
    pybind11::dict get_observation_bounds() const;  
    ~RVO2_RL_Wrapper();
    pybind11::array_t<float> get_neighbors(int agent_id) const;
    pybind11::array_t<float> get_lidar(int agent_id) const;
    

    float getLidarRange() const;

    std::vector<RVO::Vector2> getGoals() const;
    void setGoal(std::size_t agent_id, const RVO::Vector2 &goal);
    RVO::Vector2 getGoal(std::size_t agent_id) const;
    void setGoals(const std::vector<RVO::Vector2> &goals);
    void setPreferredVelocities();
    void computeDistancesToGoal();
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
  };
} // namespace RL_EXTENSIONS

#endif