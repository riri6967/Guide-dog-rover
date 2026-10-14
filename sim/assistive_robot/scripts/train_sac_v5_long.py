import os
import torch

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor

from assistive_sac_env_v5 import AssistiveNavEnvV5


MODEL_DIR = os.path.expanduser(
    "~/assistive_robot_ws/models/sac_v5"
)

LOG_DIR = os.path.expanduser(
    "~/assistive_robot_ws/logs/sac_v5"
)

os.makedirs(MODEL_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)


def main():

    print("=" * 75)
    print("ASSISTIVE SAC V5 — FULL MAP CONTINUOUS TRAINING")
    print("=" * 75)
    print("Curriculum:")
    print("  Episodes   1-80    : CORRIDOR")
    print("  Episodes  81-160   : ROOM1")
    print("  Episodes 161-260   : ROOM2")
    print("  Episodes 261+      : FULL_MAP")
    print()
    print("Total timesteps     : 1,000,000")
    print("Checkpoint interval : 25,000")
    print("Replay buffer saving : ENABLED")
    print()

    print(
        "CUDA available:",
        torch.cuda.is_available()
    )

    if torch.cuda.is_available():
        print(
            "GPU:",
            torch.cuda.get_device_name(0)
        )

    env = AssistiveNavEnvV5()
    env = Monitor(env)

    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path=MODEL_DIR,
        name_prefix="assistive_sac_v5",
        save_replay_buffer=True
    )

    model = SAC(
        "MlpPolicy",
        env,

        learning_rate=3e-4,
        buffer_size=100000,
        learning_starts=1000,

        batch_size=256,

        tau=0.005,
        gamma=0.99,

        train_freq=1,
        gradient_steps=1,

        ent_coef="auto",

        policy_kwargs=dict(
            net_arch=[256, 256]
        ),

        verbose=1,
        tensorboard_log=LOG_DIR,
        device="auto"
    )

    print("=" * 75)
    print("STARTING V5 CONTINUOUS TRAINING")
    print("=" * 75)

    model.learn(
        total_timesteps=1_000_000,
        callback=checkpoint_callback,
        reset_num_timesteps=True
    )

    final_path = os.path.join(
        MODEL_DIR,
        "assistive_sac_v5_final"
    )

    model.save(final_path)

    # Save the final replay buffer explicitly.
    replay_path = os.path.join(
        MODEL_DIR,
        "assistive_sac_v5_final_replay_buffer.pkl"
    )

    model.save_replay_buffer(
        replay_path
    )

    print()
    print("=" * 75)
    print("V5 LONG TRAINING COMPLETE")
    print("Model:", final_path + ".zip")
    print("Replay:", replay_path)
    print("=" * 75)

    env.close()


if __name__ == "__main__":
    main()
