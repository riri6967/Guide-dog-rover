import os
import torch

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor

from assistive_sac_env_v4 import AssistiveNavEnvV4


MODEL_DIR = os.path.expanduser(
    "~/assistive_robot_ws/models/sac_v4"
)

LOG_DIR = os.path.expanduser(
    "~/assistive_robot_ws/logs/sac_v4"
)

os.makedirs(MODEL_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)


def main():

    print("=" * 70)
    print("ASSISTIVE SAC V4")
    print("Fresh training — bidirectional action + heading reward")
    print("=" * 70)

    print("CUDA available:", torch.cuda.is_available())

    if torch.cuda.is_available():
        print(
            "GPU:",
            torch.cuda.get_device_name(0)
        )

    env = AssistiveNavEnvV4()
    env = Monitor(env)

    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path=MODEL_DIR,
        name_prefix="assistive_sac_v4"
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
        verbose=1,
        tensorboard_log=LOG_DIR,
        device="auto",
        policy_kwargs=dict(
            net_arch=[256, 256]
        )
    )

    print()
    print("Starting fresh V4 training")
    print("Total timesteps: 20,000")
    print("Checkpoint interval: 5,000")
    print()

    model.learn(
        total_timesteps=20000,
        callback=checkpoint_callback,
        reset_num_timesteps=True
    )

    final_path = os.path.join(
        MODEL_DIR,
        "assistive_sac_v4_final"
    )

    model.save(final_path)

    print()
    print("=" * 70)
    print("V4 TRAINING COMPLETE")
    print("Saved:", final_path + ".zip")
    print("=" * 70)

    env.close()


if __name__ == "__main__":
    main()
