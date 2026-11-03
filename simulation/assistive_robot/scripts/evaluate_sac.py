#!/usr/bin/env python3

from pathlib import Path

from stable_baselines3 import SAC

from assistive_sac_env import AssistiveNavEnv


def main():

    model_path = Path(
        '~/assistive_robot_ws/models/sac/'
        'assistive_sac_final.zip'
    ).expanduser()

    print(f'Loading model: {model_path}')

    env = AssistiveNavEnv(
        max_episode_steps=600
    )

    model = SAC.load(
        str(model_path),
        env=env,
        device='cuda'
    )

    print('Model loaded.')
    print('Starting deterministic evaluation...')

    try:

        obs, info = env.reset()

        total_reward = 0.0
        min_distance = 15.0

        for step in range(600):

            action, _ = model.predict(
                obs,
                deterministic=True
            )

            obs, reward, terminated, truncated, info = env.step(
                action
            )

            total_reward += reward

            min_distance = min(
                min_distance,
                info['min_lidar_distance']
            )

            if terminated or truncated:

                break

        print()
        print('========== EVALUATION ==========')
        print(f'Steps:              {step + 1}')
        print(f'Total reward:       {total_reward:.3f}')
        print(
            f'Final goal distance: '
            f'{info["goal_distance"]:.3f} m'
        )
        print(
            f'Min LiDAR distance: '
            f'{min_distance:.3f} m'
        )
        print(
            f'Goal reached:       '
            f'{info["goal_reached"]}'
        )
        print(
            f'Collision:          '
            f'{info["collision"]}'
        )
        print(
            f'Final linear speed: '
            f'{info["linear_velocity"]:.3f} m/s'
        )
        print(
            f'Final angular speed: '
            f'{info["angular_velocity"]:.3f} rad/s'
        )
        print('================================')

    finally:

        env.close()


if __name__ == '__main__':
    main()
