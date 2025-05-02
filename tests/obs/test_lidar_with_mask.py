# test_lidar_with_mask.py

from rvo2_rl.rvo2 import Vector2
from rvo2_rl.rl import RVO2RLWrapper, ObsMode

def make_square(center_x, center_y, half_size=0.5):
    """Returns list of Vector2 representing a square obstacle."""
    return [
        Vector2(center_x - half_size, center_y - half_size),
        Vector2(center_x + half_size, center_y - half_size),
        Vector2(center_x + half_size, center_y + half_size),
        Vector2(center_x - half_size, center_y + half_size),
    ]

def main():
    # 1) Create wrapper with LIDAR + mask enabled
    wrapper = RVO2RLWrapper(use_obs_mask=True, use_lidar=True)

    # 2) Add one agent at origin
    sim = wrapper.get_simulator()
    sim.add_agent(Vector2(0.0, 0.0))

    # 3) Add square obstacles at N, E, S, W (distance 9)
    distances = {
        "north": (0.0,  9.0),
        "east":  (9.0,  0.0),
        "south": (0.0, -9.0),
        "west":  (-9.0,  0.0),
    }
    for cx, cy in distances.values():
        sim.add_obstacle(make_square(cx, cy))
    sim.process_obstacles()

    # 4) Set goal to NE direction (10,10)
    wrapper.set_goals([Vector2(10.0, 10.0)])

    # 5) Run a few steps and capture LIDAR readings
    out_path = "lidar_masked_test_results.txt"
    with open(out_path, "w") as f:
        f.write("LIDAR readings (angle [rad], distance, mask)\n\n")
        for step in range(3):
            # Compute preferred velocities and advance
            wrapper.set_preferred_velocities()
            sim.do_step()

            # Get LIDAR array: shape (N, 3) => [angle, distance, mask]
            lidar = wrapper.get_lidar(0)  # agent_id = 0
            f.write(f"=== Step {step} ===\n")
            for idx, (angle, dist, m) in enumerate(lidar.tolist()):
                f.write(f"{idx:03d}: angle={angle:.3f} rad, dist={dist:.3f}, mask={int(m)}\n")
            f.write("\n")

    print(f"Results written to {out_path}")

if __name__ == "__main__":
    main()
