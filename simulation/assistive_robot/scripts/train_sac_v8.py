#!/usr/bin/env python3

import os

import torch

from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor

from assistive_sac_env_v8 import (
    AssistiveNavEnvV8,
    LidarCNNExtractor
)


TOTAL_TIMESTEPS = 50_000

MODEL_DIR = (
    os.path.expanduser(
        '~/assistive_robot_ws/models/sac_v8'
    )
)


def main():

    os.makedirs(
        MODEL_DIR,
        exist_ok=True
    )

    env = AssistiveNavEnvV8()

    env = Monitor(env)

    print()
    print('=' * 72)
    print('V8 SAC TRAINING')
    print('=' * 72)

    print(
        'Observation:',
        env.observation_space
    )

    print(
        'Action:',
        env.action_space
    )

    print(
        'CUDA available:',
        torch.cuda.is_available()
    )

    if torch.cuda.is_available():

        print(
            'GPU:',
            torch.cuda.get_device_name(0)
        )

    policy_kwargs = {

        'features_extractor_class':
            LidarCNNExtractor,

        'features_extractor_kwargs': {
            'features_dim': 256
        },

        'net_arch': [
            256,
            256
        ]
    }

    model = SAC(

        'MlpPolicy',

        env,

        policy_kwargs=policy_kwargs,

        learning_rate=3e-4,

        buffer_size=200_000,

        learning_starts=5_000,

        batch_size=256,

        tau=0.005,

        gamma=0.99,

        train_freq=1,

        gradient_steps=1,

        ent_coef='auto',

        verbose=1,

        tensorboard_log=(
            MODEL_DIR + '/tb'
        ),

        device='cuda'
    )

    model.learn(
        total_timesteps=TOTAL_TIMESTEPS,
        progress_bar=True
    )

    model.save(
        MODEL_DIR
        +
        '/assistive_sac_v8_final'
    )

    env.close()

    print()
    print('=' * 72)
    print('V8 TRAINING COMPLETE')
    print('=' * 72)


if __name__ == '__main__':
    main()
