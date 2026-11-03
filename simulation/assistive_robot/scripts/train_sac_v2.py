#!/usr/bin/env python3

from pathlib import Path

import torch

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback

from assistive_sac_env_v2 import AssistiveNavEnvV2


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

    env = AssistiveNavEnvV2()

    models_dir = Path(
        '~/assistive_robot_ws/models/sac_v2'
    ).expanduser()

    logs_dir = Path(
        '~/assistive_robot_ws/logs/sac_v2'
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
        save_freq=1000,
        save_path=str(models_dir),
        name_prefix='assistive_sac_v2_resume',
        save_replay_buffer=True
    )

    checkpoint = models_dir / (
        'assistive_sac_v2_15000_steps.zip'
    )

    print()
    print('Loading checkpoint:')
    print(checkpoint)
    print()

    model = SAC.load(
        str(checkpoint),
        env=env,
        device='cuda'
    )

    print(
        'Checkpoint loaded.'
    )

    print(
        'Starting timesteps:',
        model.num_timesteps
    )

    try:

        model.learn(
            total_timesteps=20000,
            callback=checkpoint_callback,
            tb_log_name='SAC_V2_RESUME',
            reset_num_timesteps=False
        )

        model.save(
            str(
                models_dir /
                'assistive_sac_v2_final_resume'
            )
        )

        print()
        print(
            'V2 resumed training finished.'
        )

        print(
            'Final timesteps:',
            model.num_timesteps
        )

    except Exception as exc:

        print()
        print(
            'TRAINING INTERRUPTED:'
        )

        print(exc)

        emergency_path = models_dir / (
            f'assistive_sac_v2_emergency_'
            f'{model.num_timesteps}_steps'
        )

        model.save(
            str(emergency_path)
        )

        print(
            'Emergency checkpoint saved to:'
        )

        print(emergency_path)

        raise

    finally:

        env.close()


if __name__ == '__main__':
    main()
