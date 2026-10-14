#!/usr/bin/env python3

import os
import torch

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor

from assistive_sac_env_v6 import AssistiveNavEnvV6


MODEL_DIR = os.path.expanduser(
    "~/assistive_robot_ws/models/sac_v6"
)

LOG_DIR = os.path.expanduser(
    "~/assistive_robot_ws/logs/sac_v6"
)

TOTAL_TIMESTEPS = 25_000
CHECKPOINT_FREQ = 25_000


def main():

    os.makedirs(MODEL_DIR, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)

    print("=" * 72)
    print("ASSISTIVE SAC V6 — LOCAL NAVIGATION TRAINING")
    print("=" * 72)
    print("Observation:        192-D (4-frame history)")
    print("Action:              2-D")
    print("Training:            25,000 steps")
    print("Checkpoints:         every 25,000 steps")
    print("Old V5 checkpoint:   NOT USED")
    print("=" * 72)

    print("CUDA available:", torch.cuda.is_available())

    if torch.cuda.is_available():
        print("GPU:", torch.cuda.get_device_name(0))

    # --------------------------------------------------------
    # ENVIRONMENT
    # --------------------------------------------------------

    env = AssistiveNavEnvV6()
    env = Monitor(env)

    print()
    print("Observation space:", env.observation_space.shape)
    print("Action space:", env.action_space.shape)
    print()

    # --------------------------------------------------------
    # SAC
    # --------------------------------------------------------

    model = SAC(
        policy="MlpPolicy",
        env=env,

        learning_rate=3e-4,

        buffer_size=200_000,
        learning_starts=5_000,

        batch_size=256,

        tau=0.005,
        gamma=0.99,

        train_freq=1,
        gradient_steps=1,

        ent_coef="auto",

        policy_kwargs=dict(
            net_arch=[256, 256, 128]
        ),

        tensorboard_log=LOG_DIR,

        verbose=1,

        device="cuda" if torch.cuda.is_available() else "cpu",

        seed=42,
    )

    # --------------------------------------------------------
    # CHECKPOINTS
    # --------------------------------------------------------

    checkpoint_callback = CheckpointCallback(
        save_freq=CHECKPOINT_FREQ,
        save_path=MODEL_DIR,
        name_prefix="assistive_sac_v6",
        save_replay_buffer=True,
        save_vecnormalize=False,
    )

    # --------------------------------------------------------
    # TRAIN
    # --------------------------------------------------------

    print()
    print("=" * 72)
    print("STARTING V6 TRAINING")
    print("=" * 72)

    try:

        model.learn(
            total_timesteps=TOTAL_TIMESTEPS,
            callback=checkpoint_callback,
            reset_num_timesteps=True,
            progress_bar=False,
        )

    except KeyboardInterrupt:

        print()
        print("[STOP] Training interrupted by user.")

    finally:

        final_path = os.path.join(
            MODEL_DIR,
            "assistive_sac_v6_final"
        )

        model.save(final_path)

        replay_path = os.path.join(
            MODEL_DIR,
            "assistive_sac_v6_final_replay_buffer.pkl"
        )

        model.save_replay_buffer(
            replay_path
        )

        env.close()

        print()
        print("=" * 72)
        print("V6 TRAINING SAVED")
        print("=" * 72)
        print("Model:", final_path)
        print("Replay:", replay_path)
        print("=" * 72)


if __name__ == "__main__":
    main()
