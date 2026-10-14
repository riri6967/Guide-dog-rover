import os
import time

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback

from assistive_sac_env_v6 import AssistiveNavEnvV6


MODEL = (
    "/home/pushkar/assistive_robot_ws/models/sac_v6/"
    "recovery_1000/assistive_sac_v6_recovery_1000_final.zip"
)

REPLAY = (
    "/home/pushkar/assistive_robot_ws/models/sac_v6/"
    "recovery_1000/assistive_sac_v6_recovery_1000_final_replay_buffer.pkl"
)

OUT = (
    "/home/pushkar/assistive_robot_ws/models/sac_v6/"
    "resume_100k"
)


def main():
    env = None

    try:
        print("\n==============================================")
        print("V6 CLEAN RESUME")
        print("Starting checkpoint: 47760")
        print("Target total timesteps: 100000")
        print("==============================================\n")

        if not os.path.exists(MODEL):
            raise FileNotFoundError(f"Model not found:\n{MODEL}")

        if not os.path.exists(REPLAY):
            raise FileNotFoundError(f"Replay buffer not found:\n{REPLAY}")

        os.makedirs(OUT, exist_ok=True)

        print("[1] Creating V6 environment...")
        env = AssistiveNavEnvV6()

        print("[2] Loading model...")
        model = SAC.load(
            MODEL,
            env=env,
            device="cuda",
        )

        print(f"[MODEL] loaded timesteps = {model.num_timesteps}")
        print(f"[MODEL] device = {model.device}")

        print("[3] Loading replay buffer...")
        model.load_replay_buffer(
            REPLAY,
            truncate_last_traj=True,
        )

        print("[REPLAY] loaded successfully")

        print("[4] Waiting for ROS/Gazebo to settle...")
        time.sleep(3.0)

        callback = CheckpointCallback(
            save_freq=5000,
            save_path=OUT,
            name_prefix="assistive_sac_v6_100k",
            save_replay_buffer=True,
        )

        print("\n==============================================")
        print("STARTING TRAINING")
        print("47760 -> 100000")
        print("==============================================\n")

        model.learn(
            total_timesteps=100000,
            reset_num_timesteps=False,
            callback=callback,
        )

        final_model = os.path.join(
            OUT,
            "assistive_sac_v6_100k_final",
        )

        final_replay = (
            final_model + "_replay_buffer.pkl"
        )

        model.save(final_model)
        model.save_replay_buffer(final_replay)

        print("\n==============================================")
        print("✅ V6 TRAINING COMPLETE")
        print("==============================================")
        print(f"Final timesteps : {model.num_timesteps}")
        print(f"Model           : {final_model}.zip")
        print(f"Replay          : {final_replay}")
        print("==============================================")

    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Saving emergency checkpoint...")

        if env is not None:
            try:
                emergency_model = os.path.join(
                    OUT,
                    f"assistive_sac_v6_interrupt_{model.num_timesteps}",
                )

                emergency_replay = (
                    emergency_model + "_replay_buffer.pkl"
                )

                model.save(emergency_model)
                model.save_replay_buffer(emergency_replay)

                print(
                    f"[SAVED] {emergency_model}.zip"
                )
                print(
                    f"[SAVED] {emergency_replay}"
                )

            except Exception as exc:
                print(
                    f"[EMERGENCY SAVE FAILED] {exc}"
                )

    finally:
        if env is not None:
            try:
                env.close()
            except Exception as exc:
                print(f"[CLOSE WARNING] {exc}")


if __name__ == "__main__":
    main()
