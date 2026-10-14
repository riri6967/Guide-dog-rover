import time

from stable_baselines3 import SAC

from assistive_sac_env_v7 import AssistiveNavEnvV7


CHECKPOINT = (
    "/home/pushkar/assistive_robot_ws/models/sac_v6/"
    "resume_100k_headless/"
    "assistive_sac_v6_headless_107760_steps.zip"
)


def main():

    print("\n==============================================")
    print("V7 CONTINUOUS MISSION TEST")
    print("==============================================")
    print(
        "Checkpoint:",
        CHECKPOINT
    )
    print(
        "Mission targets:",
        len(AssistiveNavEnvV7.MISSION_ROUTE)
    )
    print("One spawn/reset for the entire mission")
    print("Deterministic SAC policy")
    print("==============================================\n")

    env = AssistiveNavEnvV7()

    model = SAC.load(
        CHECKPOINT,
        env=None
    )

    print(
        "[MODEL] loaded:",
        model.num_timesteps
    )

    obs, info = env.reset()

    print(
        f"[START] target="
        f"{info['target_index'] + 1}/"
        f"{info['target_count']}"
    )

    total_reward = 0.0

    try:

        for step in range(
            AssistiveNavEnvV7.MAX_MISSION_STEPS
        ):

            action, _ = model.predict(
                obs,
                deterministic=True
            )

            obs, reward, terminated, truncated, info = (
                env.step(action)
            )

            total_reward += float(reward)

            if (
                info["target_changed"]
                or info["mission_complete"]
                or info["collision"]
                or info["stuck_detected"]
                or step % 50 == 0
            ):

                print(
                    f"[STEP {step + 1:04d}] "
                    f"target="
                    f"{info['target_index'] + 1}/"
                    f"{info['target_count']} "
                    f"reached="
                    f"{info['targets_reached']} "
                    f"dist="
                    f"{info['goal_distance']:.2f}m "
                    f"lidar="
                    f"{info['min_lidar_distance']:.2f}m "
                    f"path="
                    f"{info['path_length']:.2f}m "
                    f"stuck="
                    f"{info['stuck_detected']}"
                )

            if terminated or truncated:

                break

            # Small pacing delay keeps Gazebo/ROS healthy and
            # makes the GUI behavior easier to observe.
            time.sleep(0.005)

    except KeyboardInterrupt:

        print("\n[INTERRUPTED] stopping mission.")

    finally:

        env.close()

    print("\n==============================================")
    print("V7 RESULT")
    print("==============================================")
    print(
        "Targets reached:",
        info.get("targets_reached", 0),
        "/",
        info.get("target_count", 0)
    )
    print(
        "Path length:",
        f"{info.get('path_length', 0.0):.2f} m"
    )
    print(
        "Total reward:",
        f"{total_reward:.2f}"
    )

    if info.get("mission_complete", False):

        print(
            "STATUS: ✅ CONTINUOUS MISSION COMPLETE"
        )

    elif info.get("collision", False):

        print(
            "STATUS: ❌ COLLISION"
        )

    elif info.get("stuck_detected", False):

        print(
            "STATUS: ⚠️ STUCK / SUPERVISOR RECOVERY"
        )

    else:

        print(
            "STATUS: ⏱️ MISSION ENDED / TIMEOUT"
        )

    print("==============================================")


if __name__ == "__main__":
    main()
