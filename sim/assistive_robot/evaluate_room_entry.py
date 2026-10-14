import time
import numpy as np
from stable_baselines3 import SAC

from assistive_sac_env_v3 import AssistiveNavEnvV2

MODEL_PATH = "/home/pushkar/assistive_robot_ws/models/sac_v3/assistive_sac_v3_final.zip"

# One corridor goal + one goal just inside each room
TEST_GOALS = [
    ("room2", 5.20, -6.00),
]

MAX_STEPS = 600

def force_goal(env, x, y):
    env.goal_x = x
    env.goal_y = y

    prev_seq = env.ros.state_seq
    env.publish_goal_reliably()

    state = env.wait_for_new_state(prev_seq)
    env.previous_goal_distance = env.goal_distance_from_state(state)

    return np.asarray(state, dtype=np.float32)


def entered_room1(x, y):
    return y > 1.7 and -6.2 < x < -3.8


def entered_room2(x, y):
    return y < -1.7 and 3.8 < x < 6.2


env = AssistiveNavEnvV2()

print("\nLoading final 140k model...")
model = SAC.load(MODEL_PATH, env=env, device="cuda")
print("Model loaded.\n")

for name, gx, gy in TEST_GOALS:
    print("=" * 60)
    print(f"TEST: {name.upper()}  target=({gx:.2f}, {gy:.2f})")

    obs, _ = env.reset()

    # Replace randomly selected training goal with exact test goal
    obs = force_goal(env, gx, gy)

    trajectory = []
    room1_hit = False
    room2_hit = False

    for step in range(MAX_STEPS):
        action, _ = model.predict(obs, deterministic=True)

        obs, reward, terminated, truncated, info = env.step(action)

        x = float(env.ros.odom_x)
        y = float(env.ros.odom_y)

        trajectory.append((x, y))

        if entered_room1(x, y):
            room1_hit = True

        if entered_room2(x, y):
            room2_hit = True

        if terminated or truncated:
            break

    final_x = float(env.ros.odom_x)
    final_y = float(env.ros.odom_y)
    final_dist = float(np.hypot(gx - final_x, gy - final_y))

    print(f"Steps:              {step + 1}")
    print(f"Final position:     ({final_x:.2f}, {final_y:.2f})")
    print(f"Final goal dist:    {final_dist:.2f} m")
    print(f"Path length:        {env.path_length:.2f} m")
    print(f"Goal reached:       {info.get('goal_reached', False)}")
    print(f"Collision:          {info.get('collision', False)}")
    print(f"Entered Room 1:     {room1_hit}")
    print(f"Entered Room 2:     {room2_hit}")

    time.sleep(1)

env.close()

print("\nROOM ENTRY TEST COMPLETE")
