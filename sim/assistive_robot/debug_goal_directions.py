import math
import os
import time
import numpy as np
import rclpy

from stable_baselines3 import SAC
from assistive_sac_env_v3 import AssistiveNavEnvV2


MODEL_PATH = os.path.expanduser(
    "~/assistive_robot_ws/models/sac_v3/assistive_sac_v3_final.zip"
)

# Same starting position, different goal directions.
TEST_GOALS = [
    ("FORWARD",  5.0,  0.0),
    ("LEFT",     0.0,  5.0),
    ("RIGHT",    0.0, -5.0),
    ("BEHIND",  -5.0,  0.0),
]


def force_goal(env, x, y):
    env.goal_x = x
    env.goal_y = y
    env.publish_goal_reliably()

    state = env.wait_for_state()

    env.previous_goal_distance = (
        env.goal_distance_from_state(state)
    )

    return state


rclpy.init()

print("Loading 140k SAC model...")
model = SAC.load(MODEL_PATH)

env = AssistiveNavEnvV2()

print("\n==============================================")
print("        SAC GOAL-DIRECTION SANITY TEST")
print("==============================================")

for name, gx, gy in TEST_GOALS:

    print("\n----------------------------------------------")
    print(f"TEST: {name}")
    print(f"Goal: ({gx:.1f}, {gy:.1f})")

    obs, _ = env.reset()

    obs = force_goal(
        env,
        gx,
        gy
    )

    time.sleep(0.5)

    state = env.wait_for_state()

    goal_distance = env.goal_distance_from_state(state)
    goal_bearing = float(state[13]) * math.pi

    action, _ = model.predict(
        obs,
        deterministic=True
    )

    action = np.asarray(action).flatten()

    sac_linear, sac_angular = env.action_to_velocity(
        action
    )

    print(
        f"Goal distance : {goal_distance:.2f} m"
    )

    print(
        f"Goal bearing  : "
        f"{math.degrees(goal_bearing):+.1f} deg"
    )

    print(
        f"SAC action    : "
        f"[{action[0]:+.3f}, {action[1]:+.3f}]"
    )

    print(
        f"SAC command   : "
        f"linear={sac_linear:+.3f} m/s "
        f"angular={sac_angular:+.3f} rad/s"
    )

    # Let the supervisor/environment see only a very short action burst.
    for _ in range(5):
        env.step(action)

    print("5-step command burst complete.")

env.close()

rclpy.shutdown()
