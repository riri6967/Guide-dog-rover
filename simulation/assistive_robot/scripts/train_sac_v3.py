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

os.makedirs(MODEL_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)


V2_MODEL = os.path.join(
    BASE_DIR,
    "models",
    "sac_v2",
    "assistive_sac_v2_final_resume.zip"
)


TOTAL_NEW_TIMESTEPS = 100_000


def main():

    print("=" * 60)
    print("ASSISTIVE SAC V3 — MAP-WIDE AUTOMATIC TRAINING")
    print("=" * 60)

    print(
        "CUDA available:",
        torch.cuda.is_available()
    )

    if torch.cuda.is_available():
        print(
            "GPU:",
            torch.cuda.get_device_name(0)
        )

    env = AssistiveNavEnvV2()

    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path=MODEL_DIR,
        name_prefix="assistive_sac_v3",
        save_replay_buffer=True,
        save_vecnormalize=False
    )

    if os.path.exists(V2_MODEL):

        print()
        print("Loading existing V2 policy:")
        print(V2_MODEL)

        model = SAC.load(
            V2_MODEL,
            env=env,
            device="cuda" if torch.cuda.is_available() else "cpu"
        )

        print("V2 policy loaded.")
        print(
            "Starting timesteps:",
            model.num_timesteps
        )

    else:

        print()
        print("No V2 checkpoint found.")
        print("Creating a new SAC policy.")

        model = SAC(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            buffer_size=100_000,
            learning_starts=1000,
            batch_size=256,
            tau=0.005,
            gamma=0.99,
            train_freq=1,
            gradient_steps=1,
            ent_coef="auto",
            verbose=1,
            tensorboard_log=LOG_DIR,
            device="cuda" if torch.cuda.is_available() else "cpu"
        )

    try:

        print()
        print("Starting automatic curriculum training...")
        print(
            "Additional timesteps:",
            TOTAL_NEW_TIMESTEPS
        )

        model.learn(
            total_timesteps=TOTAL_NEW_TIMESTEPS,
            callback=checkpoint_callback,
            reset_num_timesteps=False,
            progress_bar=False
        )

        final_path = os.path.join(
            MODEL_DIR,
            "assistive_sac_v3_final"
        )

        model.save(final_path)

        try:
            model.save_replay_buffer(
                final_path + "_replay_buffer"
            )
        except Exception as e:
            print(
                "Replay buffer save warning:",
                e
            )

        print()
        print("=" * 60)
        print("V3 TRAINING COMPLETE")
        print(
            "Final timesteps:",
            model.num_timesteps
        )
        print(
            "Final model:",
            final_path + ".zip"
        )
        print("=" * 60)

    except KeyboardInterrupt:

        emergency_path = os.path.join(
            MODEL_DIR,
            "assistive_sac_v3_emergency"
        )

        print()
        print("Training interrupted.")
        print("Saving emergency checkpoint...")

        model.save(emergency_path)

        try:
            model.save_replay_buffer(
                emergency_path + "_replay_buffer"
            )
        except Exception as e:
            print(
                "Replay buffer save warning:",
                e
            )

        print(
            "Emergency model:",
            emergency_path + ".zip"
        )

    except Exception as e:

        emergency_path = os.path.join(
            MODEL_DIR,
            "assistive_sac_v3_error_recovery"
        )

        print()
        print("TRAINING ERROR:")
        print(e)

        try:
            model.save(emergency_path)

            model.save_replay_buffer(
                emergency_path + "_replay_buffer"
            )

            print(
                "Recovery model saved:",
                emergency_path + ".zip"
            )

        except Exception as save_error:

            print(
                "Could not save recovery model:",
                save_error
            )

        raise

    finally:

        try:
            env.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
