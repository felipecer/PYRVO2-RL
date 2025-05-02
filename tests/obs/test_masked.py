# test_rvo2rl_masked.py

from rvo2_rl.rvo2 import Vector2
from rvo2_rl.rl import RVO2RLWrapper, ObsMode

def main():
    # 1) Crear wrappers con máscara
    wrapper_cart_masked  = RVO2RLWrapper(use_obs_mask=True)                         # Cartesian + mask
    wrapper_polar_masked = RVO2RLWrapper(mode=ObsMode.Polar, use_obs_mask=True)     # Polar + mask

    # 2) Definir posiciones iniciales y metas
    positions = [(1,1), (1,9), (9,1), (9,9)]
    goals     = [(9,9), (9,1), (1,9), (1,1)]

    # 3) Añadir agentes y fijar metas
    for pos in positions:
        wrapper_cart_masked.get_simulator().add_agent(Vector2(*pos))
        wrapper_polar_masked.get_simulator().add_agent(Vector2(*pos))

    wrapper_cart_masked .set_goals([Vector2(x,y) for x,y in goals])
    wrapper_polar_masked.set_goals([Vector2(x,y) for x,y in goals])

    # 4) Abrir fichero de resultados
    out_path = "rvo2rl_masked_test_results.txt"
    with open(out_path, "w") as f:
        f.write("Masked neighbor observations (last column is mask):\n")
        f.write("(Cartesian + mask)               | (Polar + mask)\n\n")

        # 5 pasos de simulación
        for step in range(5):
            f.write(f"=== Step {step} ===\n")

            # 5.1) Compute preferred velocities
            wrapper_cart_masked .set_preferred_velocities()
            wrapper_polar_masked.set_preferred_velocities()

            # 5.2) Advance both simulators
            wrapper_cart_masked .get_simulator().do_step()
            wrapper_polar_masked.get_simulator().do_step()

            # 5.3) Registrar datos de vecinos
            for agent_id in range(len(positions)):
                cart = wrapper_cart_masked .get_neighbors(agent_id)
                polar= wrapper_polar_masked.get_neighbors(agent_id)

                f.write(f"-- Agent {agent_id} --\n")
                nrows = cart.shape[0]
                for row in range(nrows):
                    row_c = cart[row].tolist()
                    row_p = polar[row].tolist()
                    # alineación columnar
                    left  = f"{row_c!r}".ljust(45)
                    right = f"{row_p!r}"
                    f.write(f"{left} | {right}\n")
                f.write("\n")
            f.write("\n")

    print(f"Results written to {out_path}")

if __name__ == "__main__":
    main()
