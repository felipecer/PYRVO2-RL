#include "RayCastingEngine.h"
namespace RL_EXTENSIONS
{
  float RayCastingEngine::cross(const RVO::Vector2 &a, const RVO::Vector2 &b) noexcept
  {
    return a.x() * b.y() - a.y() * b.x();
  }
  void RayCastingEngine::initRays(std::size_t count, float length)
  {
    rays_.resize(count);

    const float TWO_PI = 6.2831853071795864769f;
#ifdef _OPENMP
#pragma omp parallel for simd // build table in parallel if many rays
#endif
    for (std::size_t i = 0; i < count; ++i)
    {
      float angle = TWO_PI * static_cast<float>(i) / count;
      rays_[i] = RVO::Vector2(std::cos(angle) * length,
                              std::sin(angle) * length);
    }
  }

  bool RayCastingEngine::raySegmentIntersect(const RVO::Vector2 &A, const RVO::Vector2 &B,
                                             const RVO::Vector2 &C, const RVO::Vector2 &v,
                                             float &outR) const noexcept
  {
    constexpr float EPS = 1e-7f;

    const RVO::Vector2 u = B - A; // segment direction
    const RVO::Vector2 w = C - A; // vector A → C

    const float den = cross(u, v); // 0 ⇒ parallel / colinear
    const float s_num = cross(w, v);
    const float r_num = cross(w, u);

    // ---------- parallel branch --------------------------------------
    if (std::fabs(den) < EPS)
    {
      // distinct parallels → no intersection
      if (std::fabs(s_num) > EPS)
        return false;

      // ------------ colinear case: 1‑D overlap test ----------------
      // project onto the dominant axis for better precision
      const bool useX = (std::fabs(u.x()) > std::fabs(u.y()));
      // const float uProj = useX ? u.x() : u.y();
      const float vProj = useX ? v.x() : v.y();
      const float aProj = 0.0f; // A is origin
      const float bProj = useX ? (B.x() - A.x()) : (B.y() - A.y());
      const float cProj = useX ? (C.x() - A.x()) : (C.y() - A.y());

      const float sign = (vProj >= 0.f) ? 1.f : -1.f;
      const float segMin = sign * std::fmin(aProj, bProj);
      const float segMax = sign * std::fmax(aProj, bProj);
      const float ray0 = sign * cProj;

      if (segMax < ray0 - EPS)
        return false; // behind ray

      // first overlap point (only if caller wants r)
      outR = (std::fmax(segMin, ray0) - ray0) /
             (std::fabs(vProj) + EPS);
      return true;
    }

    // ---------- regular (non‑parallel) branch ------------------------
    // Cheap range tests without division (SIMD‑friendly)

    // s = s_num / den ∈ [0,1]
    if ((den > 0.f && (s_num < 0.f || s_num > den)) ||
        (den < 0.f && (s_num > 0.f || s_num < den)))
      return false;

    // r = r_num / den ≥ 0
    if ((den > 0.f && r_num < 0.f) ||
        (den < 0.f && r_num > 0.f))
      return false;

    // Valid hit → compute r once
    outR = r_num / den;
    return true;
  }

  void RayCastingEngine::computeIntersections(
      const RVO::Vector2 &origin,
      const std::vector<std::pair<RVO::Vector2, RVO::Vector2>> &segments,
      std::vector<float> &outR,
      std::vector<int> &outSegIdx) const
  {
    const std::size_t RayCount = rays_.size();
    outR.assign(RayCount, std::numeric_limits<float>::infinity());
    outSegIdx.assign(RayCount, -1);

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (std::size_t r = 0; r < RayCount; ++r)
    {
      const RVO::Vector2 &dir = rays_[r];
      float bestR = std::numeric_limits<float>::infinity();
      int bestIx = -1;

      /* --------------------------------------------------------------
       *  (4)  SIMD‑friendly inner loop with reduction on bestR
       * -------------------------------------------------------------- */
#ifdef _OPENMP
#pragma omp simd reduction(min : bestR)
#endif
      for (std::size_t s = 0; s < segments.size(); ++s)
      {
        float rHit;
        if (raySegmentIntersect(segments[s].first,
                                segments[s].second,
                                origin, dir, rHit))
        {
          if (rHit < bestR)
          { // we still need arg‑min logic
            bestR = rHit;
            bestIx = static_cast<int>(s);
          }
        }
      }

      /* --------------------------------------------------------------
       *  (5)  Optional length‑cap: ignore hits beyond the ray’s range
       *       (each dir has magnitude == constructor ‘length’ so
       *        rHit > 1.0  means the intersection is farther than
       *        that range).
       * -------------------------------------------------------------- */
      if (bestR > 1.0f)
      { // comment out if not needed
        bestR = std::numeric_limits<float>::infinity();
        bestIx = -1;
      }

      outR[r] = bestR;
      outSegIdx[r] = bestIx;
    }
  }

  ProcessedRays RayCastingEngine::postprocessRayDistances(const std::vector<float> &rayDistances, float max_range)
  {
    ProcessedRays result;
    const std::size_t n = rayDistances.size();
    result.normalizedDistances.reserve(n);
    result.hitMask.reserve(n);

    for (float r : rayDistances)
    {
      if (std::isinf(r) || r > max_range)
      {
        result.normalizedDistances.push_back(1.0f); // far = max
        result.hitMask.push_back(0);                // no hit
      }
      else
      {
        result.normalizedDistances.push_back(r / max_range);
        result.hitMask.push_back(1); // valid hit
      }
    }

    return result;
  }
} // namespace RL_EXTENSIONS
