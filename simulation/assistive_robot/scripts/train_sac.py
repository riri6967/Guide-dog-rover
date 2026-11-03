#!/usr/bin/env python3

from pathlib import Path

import torch

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback

from assistive_sac_env import AssistiveNavEnv


def main():

    print(
        'CUDA available:',
        torch.cuda.is_available()
    )

    if torch.cuda.is_available():

        print(
            'GPU:',
            torch.cuda.get_device_name(0)
        )

    env = AssistiveNavEnv(
        max_episode_steps=600
    )

    models_dir = Path(
        '~/assistive_robot_ws/'
        'models/sac'
    ).expanduser()

    logs_dir = Path(
        '~/assistive_robot_ws/'
        'logs/sac'
    ).expanduser()

    models_dir.mkdir(
        parents=True,
        exist_ok=True
    )

    logs_dir.mkdir(
        parents=True,
        exist_ok=True
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path=str(models_dir),
        name_prefix='assistive_sac'
    )

    model = SAC(
        'MlpPolicy',
        env,
        learning_rate=3e-4,
        buffer_size=50000,
        learning_starts=500,
        batch_size=256,
        tau=0.005,
        gamma=0.99,
        train_freq=1,
        gradient_steps=1,
        ent_coef='auto',
        policy_kwargs={
            'net_arch': [256, 256]
        },
        tensorboard_log=str(logs_dir),
        verbose=1,
        device='cuda'
    )

    try:

        model.learn(
            total_timesteps=5000,
            callback=checkpoint_callback
        )

        model.save(
            str(models_dir / 'assistive_sac_final')
        )

        print(
            'Training finished.'
        )

    finally:

        env.close()


if __name__ == '__main__':
    main()
