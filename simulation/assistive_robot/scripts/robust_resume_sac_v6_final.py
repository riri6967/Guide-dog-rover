#!/usr/bin/env python3

import os
import traceback

import torch

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.monitor import Monitor

from assistive_sac_env_v6 import AssistiveNavEnvV6


# ============================================================
# CONFIGURATION
# ============================================================

MODEL_DIR = os.path.expanduser(
    "~/assistive_robot_ws/models/sac_v6"
)

LOG_DIR = os.path.expanduser(
    "~/assistive_robot_ws/logs/sac_v6/resume_contact_fixed"
)

RESUME_MODEL = os.path.join(
    MODEL_DIR,
    "assistive_sac_v6_resume_28560_steps.zip"
)

RESUME_REPLAY = os.path.join(
    MODEL_DIR,
    "assistive_sac_v6_resume_replay_buffer_28560_steps.pkl"
)

TARGET_TOTAL_STEPS = 100_000

CHECKPOINT_FREQ = 10_000

# The 28,560-step checkpoint was around the
# DOORWAY_ROOM1 curriculum.
#
# Environment reset increments the counter,
# so the next episode begins at 176.
RESUME_CURRICULUM_EPISODE = 175


def main():

    print("=" * 78)
    print("ASSISTIVE SAC V6 — CONTACT-FIXED RESUME")
    print("=" * 78)

    print(f"Checkpoint:      {RESUME_MODEL}")
    print(f"Replay:          {RESUME_REPLAY}")
    print(f"Target steps:    {TARGET_TOTAL_STEPS}")
    print(f"Resume episode:  {RESUME_CURRICULUM_EPISODE}")
    print(f"CUDA available:  {torch.cuda.is_available()}")

    if torch.cuda.is_available():
        print(
            f"GPU:             "
            f"{torch.cuda.get_device_name(0)}"
        )

    print("=" * 78)

    if not os.path.isfile(RESUME_MODEL):
        raise FileNotFoundError(
            RESUME_MODEL
        )

    if not os.path.isfile(RESUME_REPLAY):
        raise FileNotFoundError(
            RESUME_REPLAY
        )

    env_raw = None
    env = None
    model = None

    try:

        # ----------------------------------------------------
        # ENVIRONMENT
        # ----------------------------------------------------

        env_raw = AssistiveNavEnvV6()

        env_raw.curriculum_episode = (
            RESUME_CURRICULUM_EPISODE
        )

        env_raw._update_curriculum()

        print(
            f"[RESUME] Curriculum restored: "
            f"episode={env_raw.curriculum_episode} "
            f"stage={env_raw.curriculum_stage}"
        )

        env = Monitor(
            env_raw
        )

        print(
            f"[RESUME] Observation space: "
            f"{env.observation_space.shape}"
        )

        # ----------------------------------------------------
        # LOAD MODEL
        # ----------------------------------------------------

        print("[RESUME] Loading SAC model...")

        model = SAC.load(
            RESUME_MODEL,
            env=env,
            device="cuda"
        )

        print(
            f"[RESUME] Model timesteps: "
            f"{model.num_timesteps}"
        )

        # ----------------------------------------------------
        # LOAD REPLAY
        # ----------------------------------------------------

        print("[RESUME] Loading replay buffer...")

        model.load_replay_buffer(
            RESUME_REPLAY,
            truncate_last_traj=True
        )

        print(
            f"[RESUME] Replay size: "
            f"{model.replay_buffer.size()}"
        )

        # ----------------------------------------------------
        # LOGGER
        # ----------------------------------------------------

        os.makedirs(
            LOG_DIR,
            exist_ok=True
        )

        logger = configure(
            LOG_DIR,
            ["stdout", "tensorboard"]
        )

        model.set_logger(
            logger
        )

        # ----------------------------------------------------
        # CHECKPOINTS
        # ----------------------------------------------------

        checkpoint_callback = CheckpointCallback(
            save_freq=CHECKPOINT_FREQ,
            save_path=MODEL_DIR,
            name_prefix="assistive_sac_v6_contact_fixed",
            save_replay_buffer=True
        )

        print("=" * 78)
        print(
            f"[RESUME] Current timesteps: "
            f"{model.num_timesteps}"
        )
        print(
            f"[RESUME] Target timesteps:  "
            f"{TARGET_TOTAL_STEPS}"
        )
        print("=" * 78)

        if model.num_timesteps >= TARGET_TOTAL_STEPS:

            print(
                "[RESUME] Target already reached."
            )

            return

        # ----------------------------------------------------
        # TRAIN
        # ----------------------------------------------------

        print(
            "[RESUME] STARTING TRAINING..."
        )

        model.learn(
            total_timesteps=TARGET_TOTAL_STEPS,
            callback=checkpoint_callback,
            reset_num_timesteps=False,
            log_interval=1
        )

        # ----------------------------------------------------
        # FINAL SAVE
        # ----------------------------------------------------

        final_model = os.path.join(
            MODEL_DIR,
            "assistive_sac_v6_contact_fixed_100k"
        )

        final_replay = os.path.join(
            MODEL_DIR,
            "assistive_sac_v6_contact_fixed_100k_replay_buffer.pkl"
        )

        print("=" * 78)
        print(
            f"[RESUME] Finished at "
            f"{model.num_timesteps} timesteps."
        )

        print(
            f"[RESUME] Saving final model:\n"
            f"{final_model}"
        )

        model.save(
            final_model
        )

        print(
            f"[RESUME] Saving final replay:\n"
            f"{final_replay}"
        )

        model.save_replay_buffer(
            final_replay
        )

        print("=" * 78)
        print("ASSISTIVE SAC V6 — 100K COMPLETE")
        print("=" * 78)

    except KeyboardInterrupt:

        print(
            "\n[RESUME] Training interrupted."
        )

        if model is not None:

            emergency_model = os.path.join(
                MODEL_DIR,
                f"assistive_sac_v6_interrupt_{model.num_timesteps}_steps"
            )

            emergency_replay = os.path.join(
                MODEL_DIR,
                f"assistive_sac_v6_interrupt_replay_{model.num_timesteps}_steps.pkl"
            )

            print(
                f"[RESUME] Emergency model:\n"
                f"{emergency_model}"
            )

            model.save(
                emergency_model
            )

            model.save_replay_buffer(
                emergency_replay
            )

        raise

    except Exception:

        print(
            "\n[RESUME] TRAINING CRASHED"
        )

        traceback.print_exc()

        if model is not None:

            try:

                emergency_model = os.path.join(
                    MODEL_DIR,
                    f"assistive_sac_v6_crash_{model.num_timesteps}_steps"
                )

                emergency_replay = os.path.join(
                    MODEL_DIR,
                    f"assistive_sac_v6_crash_{model.num_timesteps}_steps.pkl"
                )

                print(
                    f"[RESUME] Emergency model:\n"
                    f"{emergency_model}"
                )

                model.save(
                    emergency_model
                )

                print(
                    f"[RESUME] Emergency replay:\n"
                    f"{emergency_replay}"
                )

                model.save_replay_buffer(
                    emergency_replay
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
