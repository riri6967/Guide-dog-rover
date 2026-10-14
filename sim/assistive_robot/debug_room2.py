import math
import numpy as np

from stable_baselines3 import SAC
from assistive_sac_env_v3 import AssistiveNavEnvV2


MODEL_PATH = "/home/pushkar/assistive_robot_ws/models/sac_v3/assistive_sac_v3_final.zip"

GOAL_X = 5.20
GOAL_Y = -7.00

MAX_STEPS = 300


env = AssistiveNavEnvV2()

print("\nLoading model...")

model = SAC.load(
    MODEL_PATH,
    env=env,
    device="cuda"
)

print("Model loaded.\n")


obs, _ = env.reset()

# Force exact Room 2 goal
env.goal_x = GOAL_X
env.goal_y = GOAL_Y

previous_seq = env.ros.state_seq

env.publish_goal_reliably()

obs = env.wait_for_new_state(previous_seq)

env.previous_goal_distance = (
    env.goal_distance_from_state(obs)
)

print(
    f"TARGET = ({GOAL_X:.2f}, {GOAL_Y:.2f})"
)

print("\nStarting SAC debug...\n")


last_x = float(env.ros.odom_x)
last_y = float(env.ros.odom_y)


for step in range(MAX_STEPS):

    # SAC decision
    action, _ = model.predict(
        obs,
        deterministic=True
    )

    action = np.asarray(
        action,
        dtype=np.float32
    )

    # Convert SAC action to velocity
    linear, angular = env.action_to_velocity(
        action
    )

    # Execute through the existing environment
    obs, reward, terminated, truncated, info = (
        env.step(action)
    )

    x = float(env.ros.odom_x)
    y = float(env.ros.odom_y)

    goal_distance = (
        env.goal_distance_from_state(obs)
    )

    goal_bearing = (
        float(obs[13]) * math.pi
    )

    movement = math.hypot(
        x - last_x,
        y - last_y
    )

    last_x = x
    last_y = y


    if step % 10 == 0:

        print(
            f"step={step:03d} | "
            f"pos=({x:5.2f},{y:5.2f}) | "
            f"goal={goal_distance:5.2f}m | "
            f"bearing={math.degrees(goal_bearing):6.1f}° | "
            f"SAC_v={linear:5.2f} | "
            f"SAC_w={angular:6.2f} | "
            f"LiDAR_min={env.ros.min_lidar:5.2f} | "
            f"move={movement:5.2f}"
        )


    # Detect likely stuck behaviour
    if (
        movement < 0.01
        and linear > 0.15
        and step % 5 == 0
    ):

        print(
            "   >>> SAC commands forward "
            "but rover barely moves"
        )


    if terminated or truncated:
        break


print("\n========================================")
print("SAC DEBUG COMPLETE")
print(
    f"Final position: "
    f"({env.ros.odom_x:.2f}, {env.ros.odom_y:.2f})"
)
print(
    f"Final goal distance: "
    f"{env.goal_distance_from_state(obs):.2f} m"
)
print("========================================\n")


env.close()
