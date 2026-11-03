#!/usr/bin/env python3

import time
import numpy as np
from pathlib import Path
from stable_baselines3 import SAC

from assistive_sac_env_v2 import AssistiveNavEnvV2


def main():

    model_path = str(
        Path.home() / "assistive_robot_ws/models/sac_v2/assistive_sac_v2_final"
    )

    env = AssistiveNavEnvV2()

    model = SAC.load(
        model_path,
        env=env,
        device='cuda'
    )

    episodes = 20

    successes = 0
    collisions = 0

    rewards = []
    path_lengths = []
    final_distances = []
    min_distances = []
    episode_times = []

    print('\nStarting V2 evaluation')
    print('Episodes:', episodes)

    try:

        for episode in range(1, episodes + 1):

            obs, info = env.reset()

            total_reward = 0.0
            start_time = time.monotonic()

            final_info = None

            for step in range(600):

                action, _ = model.predict(
                    obs,
                    deterministic=True
                )

                obs, reward, terminated, truncated, info = env.step(
                    action
                )

                total_reward += float(reward)
                final_info = info

                if terminated or truncated:
                    break

            elapsed = time.monotonic() - start_time

            goal_reached = bool(
                final_info['goal_reached']
            )

            collision = bool(
                final_info['collision']
            )

            if goal_reached:
                successes += 1

            if collision:
                collisions += 1

            rewards.append(total_reward)

            path_lengths.append(
                float(final_info['path_length'])
            )

            final_distances.append(
                float(final_info['goal_distance'])
            )

            min_distances.append(
                float(final_info['min_lidar_distance'])
            )

            episode_times.append(elapsed)

            print(
                f'Episode {episode:02d} | '
                f'goal=({final_info["goal_x"]:.1f},'
                f'{final_info["goal_y"]:.1f}) | '
                f'steps={step + 1} | '
                f'reward={total_reward:.2f} | '
                f'final_dist={final_info["goal_distance"]:.2f} m | '
                f'path={final_info["path_length"]:.2f} m | '
                f'min_obs={final_info["min_lidar_distance"]:.2f} m | '
                f'success={goal_reached} | '
                f'collision={collision}'
            )

        print('\n========== V2 RESULTS ==========')

        print(
            f'Success rate:          '
            f'{100.0 * successes / episodes:.1f}%'
        )

        print(
            f'Collision rate:         '
            f'{100.0 * collisions / episodes:.1f}%'
        )

        print(
            f'Mean reward:            '
            f'{np.mean(rewards):.2f}'
        )

        print(
            f'Mean path length:       '
            f'{np.mean(path_lengths):.2f} m'
        )

        print(
            f'Mean final distance:    '
            f'{np.mean(final_distances):.2f} m'
        )

        print(
            f'Mean minimum obstacle:  '
            f'{np.mean(min_distances):.2f} m'
        )

        print(
            f'Mean episode time:      '
            f'{np.mean(episode_times):.2f} s'
        )

        print('================================')

    finally:

        env.close()


if __name__ == '__main__':
    main()
