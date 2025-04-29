#ifndef RL_EXTENSIONS_RAY_CASTING_ENGINE
#define RL_EXTENSIONS_RAY_CASTING_ENGINE
#include "src/rvo2/include/Vector2.h"

#include <vector>
#include <cmath>
#include <limits>
#ifdef _OPENMP
#include <omp.h>
#endif
namespace RL_EXTENSIONS
{
  struct ProcessedRays
  {
    std::vector<float> normalizedDistances;
    std::vector<uint8_t> hitMask;
  };
  class RayCastingEngine
  {
  private:
    std::vector<RVO::Vector2> rays_;
    float length_;
#ifdef _OPENMP
#pragma omp declare simd notinbranch
#endif
    float static cross(const RVO::Vector2 &a, const RVO::Vector2 &b) noexcept;
    void initRays(std::size_t count, float length);

  public:
    RayCastingEngine(std::size_t count = 360, float length = 18.0f)
    {
      length_ = length;
      initRays(count, length);
    }
    float getRayLength() const noexcept
    {
      return length_;
    }

    ~RayCastingEngine() = default;

    const std::vector<RVO::Vector2> &getRays() const noexcept
    {
      return rays_;
    }
#ifdef _OPENMP
#pragma omp declare simd notinbranch
#endif
    bool raySegmentIntersect(const RVO::Vector2 &A, const RVO::Vector2 &B, // segment AB
                             const RVO::Vector2 &origin,                   // ray origin C
                             const RVO::Vector2 &dir,                      // pre‑baked ray
                             float &rHit) const noexcept;                  // out param

    void computeIntersections(
        const RVO::Vector2 &origin, // ray origin
        const std::vector<std::pair<RVO::Vector2, RVO::Vector2>> &segments,
        std::vector<float> &outR,
        std::vector<int> &outSegIdx) const;
    ProcessedRays postprocessRayDistances(const std::vector<float> &rayDistances, float max_range);
  };
} // namespace RVO2_RL

#endif