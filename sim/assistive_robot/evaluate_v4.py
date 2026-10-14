import os
import numpy as np

from stable_baselines3 import SAC
from assistive_sac_env_v4 import AssistiveNavEnvV4


MODEL_PATH = os.path.expanduser(
    "~/assistive_robot_ws/models/sac_v4/assistive_sac_v4_final.zip"
)

EPISODES = 10


print("=" * 70)
print("ASSISTIVE SAC V4 — 20K MODEL EVALUATION")
print("=" * 70)

model = SAC.load(
    MODEL_PATH,
    device="cpu"
)

env = AssistiveNavEnvV4()

successes = 0
collisions = 0
total_steps = 0
total_path = 0.0
rewards = []


try:

    for episode in range(1, EPISODES + 1):

        obs, info = env.reset()

        episode_reward = 0.0
        steps = 0
        final_info = {}

        while True:

            action, _ = model.predict(
                obs,
                deterministic=True
            )

            obs, reward, terminated, truncated, final_info = (
                env.step(action)
            )

            episode_reward += float(reward)
            steps += 1

            if terminated or truncated:
                break

        success = bool(
            final_info.get(
                "goal_reached",
                False
            )
        )

        collision = bool(
            final_info.get(
                "collision",
                False
            )
        )

        if success:
            successes += 1

        if collision:
            collisions += 1

        total_steps += steps
        total_path += float(
            final_info.get(
                "path_length",
                0.0
            )
        )

        rewards.append(
            episode_reward
        )

        print(
            f"\nEpisode {episode:02d}"
        )
        print(
            f"Goal: "
            f"({info.get('goal_x', 0.0):+.2f}, "
            f"{info.get('goal_y', 0.0):+.2f})"
        )
        print(
            f"Result: "
            f"{'SUCCESS' if success else 'FAILED'}"
        )
        print(
            f"Collision: "
            f"{collision}"
        )
        print(
            f"Steps: {steps}"
        )
        print(
            f"Reward: {episode_reward:.2f}"
        )
        print(
            f"Path: "
            f"{final_info.get('path_length', 0.0):.2f} m"
        )
        print(
            f"Final distance: "
            f"{final_info.get('goal_distance', 0.0):.2f} m"
        )


finally:

    env.close()


print("\n" + "=" * 70)
print("V4 EVALUATION SUMMARY")
print("=" * 70)

print(
    f"Success rate : "
    f"{successes}/{EPISODES} "
    f"({100.0 * successes / EPISODES:.1f}%)"
)

print(
    f"Collision rate: "
    f"{collisions}/{EPISODES} "
    f"({100.0 * collisions / EPISODES:.1f}%)"
)

print(
    f"Mean steps   : "
    f"{total_steps / EPISODES:.1f}"
)

print(
    f"Mean path    : "
    f"{total_path / EPISODES:.2f} m"
)

print(
    f"Mean reward  : "
    f"{np.mean(rewards):.2f}"
)

print("=" * 70)
