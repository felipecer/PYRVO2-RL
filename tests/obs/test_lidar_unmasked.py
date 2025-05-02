# test_lidar_unmasked.py

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
    # 1) Create wrapper with LIDAR enabled but no mask
    wrapper = RVO2RLWrapper(use_obs_mask=False, use_lidar=True)

    # 2) Add one agent at origin
    sim = wrapper.get_simulator()
    sim.add_agent(Vector2(0.0, 0.0))

    # 3) Add square obstacles at N, E, S, W (distance 9)
    positions = {
        "north": (0.0, 9.0),
        "east":  (9.0, 0.0),
        "south": (0.0, -9.0),
        "west":  (-9.0, 0.0),
    }
    for cx, cy in positions.values():
        sim.add_obstacle(make_square(cx, cy))
    sim.process_obstacles()

    # 4) Set goal to NE direction (10,10)
    wrapper.set_goals([Vector2(10.0, 10.0)])

    # 5) Run steps and capture angles + distances
    out_path = "lidar_unmasked_test_results.txt"
    with open(out_path, "w") as f:
        f.write("LIDAR readings (angle [rad], distance) without mask\n\n")
        for step in range(3):
            # Compute preferred velocities and advance
            wrapper.set_preferred_velocities()
            sim.do_step()

            # Get LIDAR array: shape (N, 2) => [angle, distance]
            lidar = wrapper.get_lidar(0)  # agent_id = 0
            f.write(f"=== Step {step} ===\n")
            for idx, (angle, dist) in enumerate(lidar.tolist()):
                f.write(f"{idx:03d}: angle={angle:.3f} rad, dist={dist:.3f}\n")
            f.write("\n")

    print(f"Results written to {out_path}")

if __name__ == "__main__":
    main()
