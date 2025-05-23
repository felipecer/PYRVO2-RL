from rvo2_rl.rvo2 import Vector2
from rvo2_rl.rl import RVO2RLWrapper

def main():
    # Create wrapper
    wrapper = RVO2RLWrapper()

    # Add agents with initial positions and goals
    positions = [(0, 0), (5, 5)]
    goals = [(10, 10), (0, 0)]

    for pos in positions:
        wrapper.get_simulator().add_agent(Vector2(*pos))

    wrapper.set_goals([Vector2(x, y) for x, y in goals])
    wrapper.set_preferred_velocities()
    wrapper.set_current_goals_as_initial_goals()
    wrapper.set_current_positions_as_initial_positions()

    # Run simulation and compare distances
    out_path = "normalized_vs_raw_distance_results.txt"
    with open(out_path, "w") as f:        
        f.write("Step-by-step distances (raw vs normalized):\n\n")        
        for step in range(5):            
            f.write(f"=== Step {step} ===\n")            
            # Compute preferred velocities and advance simulation
            wrapper.set_preferred_velocities()
            wrapper.get_simulator().do_step()
            # Log distances for each agent
            for agent_id in range(len(positions)):                
                raw_distance = wrapper.get_distance_to_goal(agent_id, normalized=False)
                normalized_distance = wrapper.get_distance_to_goal(agent_id, normalized=True)                
                f.write(f"Agent {agent_id}: Raw={raw_distance:.3f}, Normalized={normalized_distance:.3f}\n")

            f.write("\n")

    print(f"Results written to {out_path}")

if __name__ == "__main__":
    main()