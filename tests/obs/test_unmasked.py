# Updated test_rvo2rl.py with set_preferred_velocities() call

from rvo2_rl.rvo2 import Vector2
from rvo2_rl.rl import RVO2RLWrapper, ObsMode

def main():
    # 1) Crear wrappers
    wrapper_cart  = RVO2RLWrapper()                         # Cartesian, no mask
    wrapper_polar = RVO2RLWrapper(mode=ObsMode.Polar)       # Polar,   no mask

    # 2) Definir posiciones iniciales y metas
    positions = [(1,1), (1,9), (9,1), (9,9)]
    goals     = [(9,9), (9,1), (1,9), (1,1)]

    # 3) Añadir agentes y fijar metas
    for pos in positions:
        wrapper_cart.get_simulator().add_agent(Vector2(*pos))
        wrapper_polar.get_simulator().add_agent(Vector2(*pos))

    # Pasamos lista de Vector2
    wrapper_cart.set_goals_list ([Vector2(x,y) for x,y in goals])
    wrapper_polar.set_goals_list([Vector2(x,y) for x,y in goals])

    # 4) Abrir fichero de resultados
    out_path = "rvo2rl_test_results_unmasked.txt"
    with open(out_path, "w") as f:
        # Encabezado
        f.write("Side-by-side neighbor observations:\n")
        f.write("(Cartesian)                 | (Polar)\n\n")

        # 5 pasos de simulación
        for step in range(5):
            f.write(f"=== Step {step} ===\n")

            # 5.1) Compute preferred velocities towards goals
            wrapper_cart.set_preferred_velocities()
            wrapper_polar.set_preferred_velocities()

            # 5.2) Advance simulation one step
            wrapper_cart.get_simulator().do_step()
            wrapper_polar.get_simulator().do_step()

            # 5.3) Registrar datos de vecinos
            for agent_id in range(len(positions)):
                cart  = wrapper_cart.get_neighbors(agent_id)
                polar = wrapper_polar.get_neighbors(agent_id)

                f.write(f"-- Agent {agent_id} --\n")
                nrows = cart.shape[0]
                for row in range(nrows):
                    row_c = cart[row].tolist()
                    row_p = polar[row].tolist()
                    left  = f"{row_c!r}".ljust(40)
                    right = f"{row_p!r}"
                    f.write(f"{left} | {right}\n")
                f.write("\n")
            f.write("\n")

    print(f"Results written to {out_path}")

if __name__ == "__main__":
    main()
