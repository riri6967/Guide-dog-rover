#!/usr/bin/env python3

import os
import traceback

import torch

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.monitor import Monitor

from assistive_sac_env_v6 import AssistiveNavEnvV6


MODEL_DIR = os.path.expanduser(
    "~/assistive_robot_ws/models/sac_v6"
)

LOG_DIR = os.path.expanduser(
    "~/assistive_robot_ws/logs/sac_v6/resume_46731_contact_fixed"
)

RESUME_MODEL = os.path.join(
    MODEL_DIR,
    "assistive_sac_v6_crash_46760.zip"
)

RESUME_REPLAY = os.path.join(
    MODEL_DIR,
    "assistive_sac_v6_crash_46760.pkl"
)

TARGET_TOTAL_STEPS = 100_000

CHECKPOINT_FREQ = 10_000

# Episode 486 completed just before the failed reset.
# env.reset() increments this, so the next episode is 487.
RESUME_CURRICULUM_EPISODE = 487


def main():

    print("=" * 78)
    print("ASSISTIVE SAC V6 — RESUME FROM 46,731")
    print("=" * 78)

    print("Model:", RESUME_MODEL)
    print("Replay:", RESUME_REPLAY)
    print("Target:", TARGET_TOTAL_STEPS)
    print("CUDA:", torch.cuda.is_available())

    if torch.cuda.is_available():
        print(
            "GPU:",
            torch.cuda.get_device_name(0)
        )

    print("=" * 78)

    if not os.path.isfile(RESUME_MODEL):
        raise FileNotFoundError(RESUME_MODEL)

    if not os.path.isfile(RESUME_REPLAY):
        raise FileNotFoundError(RESUME_REPLAY)

    env_raw = None
    env = None
    model = None

    try:

        env_raw = AssistiveNavEnvV6()

        env_raw.curriculum_episode = (
            RESUME_CURRICULUM_EPISODE
        )

        env_raw._update_curriculum()

        print(
            f"[RESUME] Curriculum: "
            f"episode={env_raw.curriculum_episode} "
            f"stage={env_raw.curriculum_stage}"
        )

        env = Monitor(env_raw)

        model = SAC.load(
            RESUME_MODEL,
            env=env,
            device="cuda"
        )

        print(
            "[RESUME] Model timesteps:",
            model.num_timesteps
        )

        model.load_replay_buffer(
            RESUME_REPLAY,
            truncate_last_traj=True
        )

        print(
            "[RESUME] Replay size:",
            model.replay_buffer.size()
        )

        os.makedirs(
            LOG_DIR,
            exist_ok=True
        )

        logger = configure(
            LOG_DIR,
            ["stdout", "tensorboard"]
        )

        model.set_logger(logger)

        checkpoint_callback = CheckpointCallback(
            save_freq=CHECKPOINT_FREQ,
            save_path=MODEL_DIR,
            name_prefix="assistive_sac_v6_contact_fixed",
            save_replay_buffer=True
        )

        print("=" * 78)
        print("STARTING RESUME")
        print(
            f"Current: {model.num_timesteps}"
        )
        print(
            f"Target : {TARGET_TOTAL_STEPS}"
        )
        print("=" * 78)

        model.learn(
            total_timesteps=TARGET_TOTAL_STEPS,
            callback=checkpoint_callback,
            reset_num_timesteps=False,
            log_interval=1
        )

        final_model = os.path.join(
            MODEL_DIR,
            "assistive_sac_v6_contact_fixed_100k"
        )

        final_replay = os.path.join(
            MODEL_DIR,
            "assistive_sac_v6_contact_fixed_100k_replay.pkl"
        )

        print("=" * 78)
        print(
            "Training finished:",
            model.num_timesteps
        )

        model.save(final_model)
        model.save_replay_buffer(final_replay)

        print("FINAL MODEL:", final_model)
        print("FINAL REPLAY:", final_replay)
        print("=" * 78)

    except KeyboardInterrupt:

        print("\n[RESUME] INTERRUPTED")

        if model is not None:

            path = os.path.join(
                MODEL_DIR,
                f"assistive_sac_v6_interrupt_{model.num_timesteps}"
            )

            replay = os.path.join(
                MODEL_DIR,
                f"assistive_sac_v6_interrupt_{model.num_timesteps}.pkl"
            )

            model.save(path)
            model.save_replay_buffer(replay)

            print("Emergency model:", path)
            print("Emergency replay:", replay)

        raise

    except Exception:

        print("\n[RESUME] CRASHED")
        traceback.print_exc()

        if model is not None:

            try:

                path = os.path.join(
                    MODEL_DIR,
                    f"assistive_sac_v6_crash_{model.num_timesteps}"
                )

                replay = os.path.join(
                    MODEL_DIR,
                    f"assistive_sac_v6_crash_{model.num_timesteps}.pkl"
                )

                model.save(path)
                model.save_replay_buffer(replay)

                print(
                    "Emergency model:",
                    path
                )

                print(
                    "Emergency replay:",
                    replay
                )

            except Exception:
                traceback.print_exc()

        raise

    finally:

        if env is not None:

            try:
                env.close()
            except Exception:
                traceback.print_exc()


if __name__ == "__main__":
    main()
