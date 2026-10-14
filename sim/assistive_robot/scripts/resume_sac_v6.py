#!/usr/bin/env python3

import os
import torch

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor

from assistive_sac_env_v6 import AssistiveNavEnvV6


MODEL_PATH = os.path.expanduser(
    "~/assistive_robot_ws/models/sac_v6/assistive_sac_v6_final.zip"
)

REPLAY_PATH = os.path.expanduser(
    "~/assistive_robot_ws/models/sac_v6/"
    "assistive_sac_v6_final_replay_buffer.pkl"
)

MODEL_DIR = os.path.expanduser(
    "~/assistive_robot_ws/models/sac_v6"
)

TARGET_TOTAL_STEPS = 25_000
RESUME_CURRICULUM_EPISODE = 100


def main():

    print("=" * 72)
    print("ASSISTIVE SAC V6 — RESUME TRAINING")
    print("=" * 72)

    env = AssistiveNavEnvV6()

    # The old run completed the corridor curriculum.
    # The next reset must begin at Episode 101.
    env.curriculum_episode = RESUME_CURRICULUM_EPISODE
    env._update_curriculum()

    print(
        f"Curriculum restored: "
        f"episode={env.curriculum_episode} "
        f"stage={env.curriculum_stage}"
    )

    env = Monitor(env)

    model = SAC.load(
        MODEL_PATH,
        env=env,
        device="cuda" if torch.cuda.is_available() else "cpu"
    )

    model.load_replay_buffer(
        REPLAY_PATH,
        truncate_last_traj=True
    )

    remaining = max(
        0,
        TARGET_TOTAL_STEPS - model.num_timesteps
    )

    print(f"Loaded timesteps: {model.num_timesteps}")
    print(f"Remaining pilot steps: {remaining}")
    print(f"Observation: {model.observation_space.shape}")
    print(f"Action: {model.action_space.shape}")
    print("=" * 72)

    if remaining == 0:
        print("Pilot already reached target.")
        env.close()
        return

    checkpoint_callback = CheckpointCallback(
        save_freq=5_000,
        save_path=MODEL_DIR,
        name_prefix="assistive_sac_v6_resume",
        save_replay_buffer=True
    )

    try:

        model.learn(
            total_timesteps=TARGET_TOTAL_STEPS,
            callback=checkpoint_callback,
            reset_num_timesteps=False,
            progress_bar=False
        )

        final_path = os.path.join(
            MODEL_DIR,
            "assistive_sac_v6_25000"
        )

        replay_path = os.path.join(
            MODEL_DIR,
            "assistive_sac_v6_25000_replay_buffer.pkl"
        )

        model.save(final_path)
        model.save_replay_buffer(replay_path)

        print("=" * 72)
        print("V6 25K PILOT COMPLETE")
        print("=" * 72)
        print("Total timesteps:", model.num_timesteps)
        print("Model:", final_path)
        print("Replay:", replay_path)
        print("=" * 72)

    except KeyboardInterrupt:

        print("\n[STOP] Training interrupted.")

        model.save(
            os.path.join(
                MODEL_DIR,
                "assistive_sac_v6_interrupted"
            )
        )

        model.save_replay_buffer(
            os.path.join(
                MODEL_DIR,
                "assistive_sac_v6_interrupted_replay_buffer.pkl"
            )
        )

    finally:
        env.close()


if __name__ == "__main__":
    main()
