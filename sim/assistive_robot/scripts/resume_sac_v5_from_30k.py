#!/usr/bin/env python3

import os
import traceback

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.monitor import Monitor

from assistive_sac_env_v5 import AssistiveNavEnvV5


MODEL_DIR = os.path.expanduser(
    "~/assistive_robot_ws/models/sac_v5"
)

LOG_DIR = os.path.expanduser(
    "~/assistive_robot_ws/logs/sac_v5/resume_30k"
)

MODEL_PATH = os.path.join(
    MODEL_DIR,
    "assistive_sac_v5_30000_steps.zip"
)

REPLAY_PATH = os.path.join(
    MODEL_DIR,
    "assistive_sac_v5_replay_buffer_30000_steps.pkl"
)

TARGET_TIMESTEPS = 1_000_000
CHECKPOINT_FREQ = 5_000

# Last curriculum episode reached before the reset failure.
# reset() increments this before starting the next episode.
RESUME_EPISODE = 111


def main():

    print("=" * 75)
    print("ASSISTIVE SAC V5 — RESUME FROM 30K")
    print("=" * 75)

    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(MODEL_PATH)

    if not os.path.exists(REPLAY_PATH):
        raise FileNotFoundError(REPLAY_PATH)

    # --------------------------------------------------------
    # Environment
    # --------------------------------------------------------

    env_raw = AssistiveNavEnvV5()

    # Restore curriculum position.
    env_raw.curriculum_episode = RESUME_EPISODE
    env_raw._update_curriculum()

    print(
        f"[RESUME] Curriculum episode = "
        f"{env_raw.curriculum_episode}"
    )

    print(
        f"[RESUME] Curriculum stage = "
        f"{env_raw.curriculum_stage}"
    )

    env = Monitor(env_raw)

    # --------------------------------------------------------
    # Load trained SAC model
    # --------------------------------------------------------

    print("[RESUME] Loading 30k SAC checkpoint...")

    model = SAC.load(
        MODEL_PATH,
        env=env,
        device="cuda"
    )

    print(
        f"[RESUME] Model timesteps = "
        f"{model.num_timesteps}"
    )

    # --------------------------------------------------------
    # Load replay buffer
    # --------------------------------------------------------

    print("[RESUME] Loading replay buffer...")

    model.load_replay_buffer(
        REPLAY_PATH,
        truncate_last_traj=True
    )

    print(
        f"[RESUME] Replay buffer size = "
        f"{model.replay_buffer.size()}"
    )

    # --------------------------------------------------------
    # Logging
    # --------------------------------------------------------

    os.makedirs(LOG_DIR, exist_ok=True)

    logger = configure(
        LOG_DIR,
        ["stdout", "tensorboard"]
    )

    model.set_logger(logger)

    # --------------------------------------------------------
    # Checkpoints
    # --------------------------------------------------------

    callback = CheckpointCallback(
        save_freq=CHECKPOINT_FREQ,
        save_path=MODEL_DIR,
        name_prefix="assistive_sac_v5",
        save_replay_buffer=True
    )

    # --------------------------------------------------------
    # Continue training
    # IMPORTANT:
    #
    # reset_num_timesteps=False means SB3 keeps the existing
    # 30,000 timestep counter.
    #
    # Therefore total_timesteps MUST be 1,000,000, not 970,000.
    # --------------------------------------------------------

    print("=" * 75)
    print(
        f"[RESUME] Starting at {model.num_timesteps} steps"
    )
    print(
        f"[RESUME] Training until {TARGET_TIMESTEPS} steps"
    )
    print("=" * 75)

    try:

        model.learn(
            total_timesteps=TARGET_TIMESTEPS,
            callback=callback,
            reset_num_timesteps=False,
            log_interval=1
        )

    except KeyboardInterrupt:

        print("\n[RESUME] Training interrupted.")

        steps = model.num_timesteps

        emergency_model = os.path.join(
            MODEL_DIR,
            f"assistive_sac_v5_interrupt_{steps}"
        )

        emergency_replay = os.path.join(
            MODEL_DIR,
            f"assistive_sac_v5_replay_buffer_interrupt_{steps}.pkl"
        )

        print(
            f"[RESUME] Saving model: {emergency_model}"
        )

        model.save(emergency_model)

        print(
            f"[RESUME] Saving replay buffer: {emergency_replay}"
        )

        model.save_replay_buffer(
            emergency_replay
        )

        raise

    except Exception:

        print("\n[RESUME] TRAINING CRASHED")
        traceback.print_exc()

        steps = model.num_timesteps

        emergency_model = os.path.join(
            MODEL_DIR,
            f"assistive_sac_v5_crash_{steps}"
        )

        emergency_replay = os.path.join(
            MODEL_DIR,
            f"assistive_sac_v5_replay_buffer_crash_{steps}.pkl"
        )

        try:
            model.save(emergency_model)
            print(
                f"[RESUME] Emergency model saved: "
                f"{emergency_model}"
            )
        except Exception:
            traceback.print_exc()

        try:
            model.save_replay_buffer(emergency_replay)
            print(
                f"[RESUME] Emergency replay saved: "
                f"{emergency_replay}"
            )
        except Exception:
            traceback.print_exc()

        raise

    else:

        final_model = os.path.join(
            MODEL_DIR,
            "assistive_sac_v5_continued_final"
        )

        final_replay = os.path.join(
            MODEL_DIR,
            "assistive_sac_v5_replay_buffer_continued_final.pkl"
        )

        model.save(final_model)
        model.save_replay_buffer(final_replay)

        print("=" * 75)
        print(
            f"[RESUME] COMPLETE: "
            f"{model.num_timesteps} total timesteps"
        )
        print("=" * 75)

    finally:
        env_raw.close()


if __name__ == "__main__":
    main()
