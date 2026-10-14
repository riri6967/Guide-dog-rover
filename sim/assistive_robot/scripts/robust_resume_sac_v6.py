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

MODEL_PATH = os.path.join(
    MODEL_DIR,
    "assistive_sac_v6_resume_28560_steps.zip"
)

REPLAY_PATH = os.path.join(
    MODEL_DIR,
    "assistive_sac_v6_resume_replay_buffer_28560_steps.pkl"
)

TARGET_TOTAL_TIMESTEPS = 100_000

# Latest observed checkpoint was still in the ROOM1 doorway
# curriculum. Episode 176 is where the previous run failed.
RESUME_CURRICULUM_EPISODE = 175


def save_checkpoint(model, name):

    model_path = os.path.join(
        MODEL_DIR,
        name
    )

    replay_path = (
        model_path
        + "_replay_buffer.pkl"
    )

    model.save(model_path)
    model.save_replay_buffer(replay_path)

    print("Saved model:", model_path)
    print("Saved replay:", replay_path)


def main():

    if not os.path.isfile(MODEL_PATH):
        raise FileNotFoundError(
            f"Missing model:\\n{MODEL_PATH}"
        )

    if not os.path.isfile(REPLAY_PATH):
        raise FileNotFoundError(
            f"Missing replay buffer:\\n{REPLAY_PATH}"
        )

    print("=" * 72)
    print("ASSISTIVE SAC V6 — ROBUST RESUME")
    print("=" * 72)
    print("Model:", MODEL_PATH)
    print("Replay:", REPLAY_PATH)
    print("Target:", TARGET_TOTAL_TIMESTEPS)
    print("=" * 72)

    env = AssistiveNavEnvV6()
    env = Monitor(env)

    model = SAC.load(
        MODEL_PATH,
        env=env,
        device=(
            "cuda"
            if torch.cuda.is_available()
            else "cpu"
        ),
    )

    model.load_replay_buffer(
        REPLAY_PATH,
        truncate_last_traj=True,
    )

    base_env = (
        model.get_env()
        .envs[0]
        .unwrapped
    )

    base_env.curriculum_episode = (
        RESUME_CURRICULUM_EPISODE
    )

    base_env._update_curriculum()

    print()
    print(
        "[RESUME] Loaded timesteps:",
        model.num_timesteps,
    )

    print(
        "[RESUME] Replay transitions:",
        model.replay_buffer.size(),
    )

    print(
        "[RESUME] Curriculum:",
        base_env.curriculum_episode,
        "->",
        base_env.curriculum_stage,
    )

    print(
        "[RESUME] Observation:",
        model.observation_space.shape,
    )

    print(
        "[RESUME] Action:",
        model.action_space.shape,
    )

    remaining = (
        TARGET_TOTAL_TIMESTEPS
        - model.num_timesteps
    )

    print(
        "[RESUME] Additional steps needed:",
        max(0, remaining),
    )

    if remaining <= 0:

        print("Target already reached.")
        env.close()
        return

    callback = CheckpointCallback(
        save_freq=5_000,
        save_path=MODEL_DIR,
        name_prefix="assistive_sac_v6_resume",
        save_replay_buffer=True,
        save_vecnormalize=False,
    )

    try:

        print("=" * 72)
        print("STARTING V6 TRAINING")
        print("=" * 72)

        # IMPORTANT:
        # reset_num_timesteps=False preserves the existing
        # 28,560 timestep counter. Therefore total_timesteps
        # is the FINAL target, not the number of extra steps.

        model.learn(
            total_timesteps=TARGET_TOTAL_TIMESTEPS,
            callback=callback,
            reset_num_timesteps=False,
            progress_bar=False,
        )

        print()
        print("=" * 72)
        print("V6 100K TRAINING COMPLETE")
        print("=" * 72)

        save_checkpoint(
            model,
            "assistive_sac_v6_100k"
        )

        print(
            "Final timesteps:",
            model.num_timesteps
        )

    except KeyboardInterrupt:

        print()
        print("=" * 72)
        print("V6 TRAINING STOPPED BY USER")
        print("=" * 72)

        save_checkpoint(
            model,
            f"assistive_sac_v6_interrupted_{model.num_timesteps}"
        )

    except Exception as exc:

        print()
        print("=" * 72)
        print("V6 TRAINING CRASHED")
        print("=" * 72)
        print(exc)
        print("=" * 72)

        save_checkpoint(
            model,
            f"assistive_sac_v6_crash_{model.num_timesteps}"
        )

        raise

    finally:

        env.close()


if __name__ == "__main__":
    main()
