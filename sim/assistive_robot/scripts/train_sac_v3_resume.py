import os
import torch

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback

from assistive_sac_env_v3 import AssistiveNavEnvV2


BASE_DIR = os.path.expanduser("~/assistive_robot_ws")

MODEL_DIR = os.path.join(
    BASE_DIR,
    "models",
    "sac_v3"
)

LOG_DIR = os.path.join(
    BASE_DIR,
    "logs",
    "sac_v3"
)

MODEL_PATH = os.path.join(
    MODEL_DIR,
    "assistive_sac_v3_40000_steps.zip"
)

REPLAY_PATH = os.path.join(
    MODEL_DIR,
    "assistive_sac_v3_replay_buffer_40000_steps.pkl"
)

os.makedirs(MODEL_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)

ADDITIONAL_TIMESTEPS = 100_000


def main():

    print("=" * 70)
    print("ASSISTIVE SAC V3 — RESUME FROM 40K")
    print("=" * 70)

    print("CUDA available:", torch.cuda.is_available())

    if torch.cuda.is_available():
        print(
            "GPU:",
            torch.cuda.get_device_name(0)
        )

    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(MODEL_PATH)

    if not os.path.exists(REPLAY_PATH):
        raise FileNotFoundError(REPLAY_PATH)

    env = AssistiveNavEnvV2()

    # Continue directly in the full-map stage.
    # The previous run reached ~40k timesteps after already
    # progressing through the curriculum.
    env.episode_count = 300

    print()
    print("Loading policy:")
    print(MODEL_PATH)

    model = SAC.load(
        MODEL_PATH,
        env=env,
        device="cuda" if torch.cuda.is_available() else "cpu"
    )

    print("Policy loaded.")
    print("Starting timesteps:", model.num_timesteps)

    print()
    print("Loading replay buffer:")
    print(REPLAY_PATH)

    model.load_replay_buffer(
        REPLAY_PATH
    )

    print(
        "Replay buffer size:",
        model.replay_buffer.size()
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path=MODEL_DIR,
        name_prefix="assistive_sac_v3",
        save_replay_buffer=True
    )

    try:

        print()
        print("=" * 70)
        print("STARTING AUTOMATIC FULL-MAP TRAINING")
        print("Additional timesteps:", ADDITIONAL_TIMESTEPS)
        print("=" * 70)

        model.learn(
            total_timesteps=ADDITIONAL_TIMESTEPS,
            callback=checkpoint_callback,
            reset_num_timesteps=False,
            progress_bar=False
        )

        final_path = os.path.join(
            MODEL_DIR,
            "assistive_sac_v3_final"
        )

        model.save(final_path)

        model.save_replay_buffer(
            final_path + "_replay_buffer"
        )

        print()
        print("=" * 70)
        print("V3 TRAINING COMPLETE")
        print("Final timesteps:", model.num_timesteps)
        print("Final model:", final_path + ".zip")
        print("=" * 70)

    except KeyboardInterrupt:

        path = os.path.join(
            MODEL_DIR,
            "assistive_sac_v3_interrupted"
        )

        print()
        print("Training interrupted.")
        print("Saving current policy...")

        model.save(path)

        model.save_replay_buffer(
            path + "_replay_buffer"
        )

        print("Saved:", path + ".zip")

    except Exception as e:

        path = os.path.join(
            MODEL_DIR,
            "assistive_sac_v3_recovery"
        )

        print()
        print("TRAINING ERROR:")
        print(e)

        model.save(path)

        model.save_replay_buffer(
            path + "_replay_buffer"
        )

        print("Recovery model:", path + ".zip")

        raise

    finally:

        env.close()


if __name__ == "__main__":
    main()
