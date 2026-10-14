import os
import numpy as np
from stable_baselines3 import SAC

from assistive_sac_env_v6 import AssistiveNavEnvV6


MODEL = (
    "/home/pushkar/assistive_robot_ws/models/sac_v6/"
    "resume_100k_headless/assistive_sac_v6_headless_107760_steps.zip"
)

STAGES = [
    ("CORRIDOR_LOCAL", 0),
    ("DOORWAY_ROOM1", 100),
    ("ROOM1_LOCAL", 200),
    ("DOORWAY_ROOM2", 300),
    ("ROOM2_LOCAL", 400),
    ("MIXED_LOCAL", 500),
]

EPISODES_PER_STAGE = 10


env = None

try:
    print("\n==============================================")
    print("V6 POLICY EVALUATION")
    print("Checkpoint: 107760")
    print("60 episodes / all curriculum stages")
    print("Deterministic policy")
    print("==============================================\n")

    env = AssistiveNavEnvV6()

    model = SAC.load(
        MODEL,
        env=env,
        device="cuda",
    )

    print(
        f"[MODEL] loaded: {model.num_timesteps} timesteps"
    )

    results = []

    for stage_name, episode_base in STAGES:

        print("\n----------------------------------------------")
        print(f"STAGE: {stage_name}")
        print("----------------------------------------------")

        for k in range(EPISODES_PER_STAGE):

            # reset() increments the episode counter internally,
            # so base=100 produces episode 101, etc.
            env.curriculum_episode = episode_base

            obs, info = env.reset()

            ep_reward = 0.0
            ep_steps = 0
            reason = "UNKNOWN"

            while ep_steps < 500:

                action, _ = model.predict(
                    obs,
                    deterministic=True,
                )

                obs, reward, terminated, truncated, info = env.step(
                    action
                )

                ep_reward += float(reward)
                ep_steps += 1

                if terminated or truncated:

                    if info.get("contact_collision", False):
                        reason = "COLLISION"

                    elif info.get("goal_reached", False):
                        reason = "SUCCESS"

                    else:
                        reason = "OTHER"

                    break

            results.append(
                {
                    "stage": stage_name,
                    "steps": ep_steps,
                    "reward": ep_reward,
                    "reason": reason,
                }
            )

            print(
                f"[{stage_name:16s}] "
                f"{k+1:02d}/10 "
                f"steps={ep_steps:3d} "
                f"reward={ep_reward:8.2f} "
                f"{reason}"
            )

    print("\n==============================================")
    print("EVALUATION SUMMARY")
    print("==============================================")

    total = len(results)
    successes = sum(
        r["reason"] == "SUCCESS" for r in results
    )
    collisions = sum(
        r["reason"] == "COLLISION" for r in results
    )

    print(f"Total episodes : {total}")
    print(f"Successes      : {successes}")
    print(f"Collisions     : {collisions}")
    print(
        f"Success rate   : "
        f"{100.0 * successes / total:.1f}%"
    )
    print(
        f"Collision rate : "
        f"{100.0 * collisions / total:.1f}%"
    )

    print("\n===== BY STAGE =====")

    for stage_name, _ in STAGES:

        subset = [
            r for r in results
            if r["stage"] == stage_name
        ]

        s = sum(
            r["reason"] == "SUCCESS"
            for r in subset
        )

        c = sum(
            r["reason"] == "COLLISION"
            for r in subset
        )

        mean_steps = np.mean(
            [r["steps"] for r in subset]
        )

        mean_reward = np.mean(
            [r["reward"] for r in subset]
        )

        print(
            f"{stage_name:16s} "
            f"success={s:2d}/10 "
            f"collision={c:2d}/10 "
            f"mean_steps={mean_steps:6.1f} "
            f"mean_reward={mean_reward:8.2f}"
        )

    print("\n==============================================")
    print("✅ EVALUATION COMPLETE")
    print("==============================================")

finally:
    if env is not None:
        try:
            env.close()
        except Exception as exc:
            print("[CLOSE WARNING]", exc)
