from rvo2_rl.rvo2 import Vector2
from rvo2_rl.rl import RVO2RLWrapper


def main():
    print("1. Creating wrapper...")
    wrapper = RVO2RLWrapper()

    print("2. Getting simulator...")
    sim = wrapper.get_simulator()

    print("3. Adding agent...")
    pos = Vector2(0.0, 0.0)
    wrapper.add_agent(pos)

    print("4. Setting goals...")
    goals = [Vector2(5.0, 5.0)]
    wrapper.set_goals(goals)

    print("5. Initializing...")
    wrapper.initialize()
    print("Initialize test passed!")


if __name__ == "__main__":
    main()
