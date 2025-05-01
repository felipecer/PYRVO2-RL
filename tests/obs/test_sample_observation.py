# test_get_observation.py

import numpy as np
from rvo2_rl.rvo2 import Vector2
from rvo2_rl.rl import RVO2RLWrapper, ObsMode

def make_square(cx, cy, half=0.1):
    return [
        Vector2(cx - half, cy - half),
        Vector2(cx + half, cy - half),
        Vector2(cx + half, cy + half),
        Vector2(cx - half, cy + half),
    ]

def run_test(name, **cfg):
    print(f"\n=== Test: {name} ===")
    # 1) Instantiate wrapper with arbitrary but distinct parameters
    wrapper = RVO2RLWrapper(
        time_step=0.5,
        neighbor_dist=10.0,
        max_neighbors=3,
        time_horizon=5.0,
        time_horizon_obst=5.0,
        radius=0.5,
        max_speed=1.5,
        velocity=Vector2(0, 0),
        mode=cfg.get("mode", ObsMode.Cartesian),
        use_obs_mask=cfg.get("use_obs_mask", False),
        use_lidar=cfg.get("use_lidar", False),
        lidar_count=cfg.get("lidar_count", 8),
        lidar_range=cfg.get("lidar_range", 5.0),
    )

    sim = wrapper.get_simulator()
    # 2) Add single agent at origin
    sim.add_agent(Vector2(0.0, 0.0))

    # 3) If LIDAR is enabled, add four square obstacles at cardinal points
    if cfg.get("use_lidar", False):
        rng = cfg.get("lidar_range", 5.0)
        for dx, dy in [(rng, 0), (0, rng), (-rng, 0), (0, -rng)]:
            sim.add_obstacle(make_square(dx, dy))
        sim.process_obstacles()

    # 4) Give it a trivial goal so dist_to_goal vectors are non‐zero
    wrapper.set_goals([Vector2(1.0, 1.0)])

    # 5) Compute preferred vel & step once
    wrapper.set_preferred_velocities()
    sim.do_step()

    # 6) Fetch bounds & a sample observation
    bounds = wrapper.get_observation_bounds()
    obs    = wrapper.get_observation(0)
    print(obs)
    low  = np.asarray(bounds["low"],  dtype=np.float32)
    high = np.asarray(bounds["high"], dtype=np.float32)
    info = bounds["info"]
    mode = bounds["mode"]

    # 7) Quick sanity checks & print
    print(" mode reported     :", mode)
    print(" obs length        :", obs.shape[0], "== bounds?", low.shape[0])
    print(" low/high shapes   :", low.shape, high.shape)
    print(" info lines        :", len(info))
    for i, line in enumerate(info):
        print(f"   [{i:02d}] {line}")
    print(" first 10 obs vals :", obs[:10].tolist())

# run a few configurations
if __name__ == "__main__":
    tests = [
        ("Cartesian, no mask, no lidar",
         dict(mode=ObsMode.Cartesian, use_obs_mask=False, use_lidar=False)),
        ("Cartesian, mask, no lidar",
         dict(mode=ObsMode.Cartesian, use_obs_mask=True,  use_lidar=False)),
        ("Cartesian, no mask, with lidar",
         dict(mode=ObsMode.Cartesian, use_obs_mask=False, use_lidar=True)),
        ("Cartesian, mask, with lidar",
         dict(mode=ObsMode.Cartesian, use_obs_mask=True,  use_lidar=True)),
        ("Polar, no mask, no lidar",
         dict(mode=ObsMode.Polar,     use_obs_mask=False, use_lidar=False)),
        ("Polar, mask, with lidar",
         dict(mode=ObsMode.Polar,     use_obs_mask=True,  use_lidar=True)),
    ]

    for name, cfg in tests:
        run_test(name, **cfg)
