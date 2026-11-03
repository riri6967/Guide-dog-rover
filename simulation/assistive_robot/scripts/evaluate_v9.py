#!/usr/bin/env python3

import argparse
import math
import os
import time

import numpy as np
from stable_baselines3 import SAC

from assistive_sac_env_v9 import AssistiveNavEnvV9


EVAL_GOALS = [
    (-5.0, 4.0),
    (5.0, -4.0),
    (-3.0, 0.0),
    (3.0, 4.0),
    (5.0, 2.0),
    (-5.0, 2.5),
]


class EvalEnv(AssistiveNavEnvV9):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.goal_pool = list(EVAL_GOALS)
        self.eval_goal_index = 0

    def _select_goal(self):
        # Deterministic mission order for fair checkpoint comparison.
        goal = self.goal_pool[
            self.eval_goal_index % len(self.goal_pool)
        ]
        self.eval_goal_index += 1

        path, length = self.planner.plan(
            self._start,
            goal,
        )

        if length <= 2.0:
            raise RuntimeError(
                f"Invalid evaluation goal {goal}, path={length:.2f}m"
            )

        return goal, path, length


def run_evaluation(model_path, episodes):
    print("\n" + "=" * 78)
    print("V9 POLICY EVALUATION")
    print("=" * 78)
    print(f"Model: {model_path}")
    print(f"Episodes: {episodes}")
    print("=" * 78)

    env = EvalEnv(max_episode_steps=2500)

    try:
        model = SAC.load(
            model_path,
            env=None,
            device="cuda",
        )

        successes = 0
        collisions = 0
        timeouts = 0

        episode_steps = []
        episode_rewards = []
        actual_distances = []
        global_paths = []
        final_goal_errors = []
        clearances = []

        for ep in range(episodes):
            obs, info = env.reset()

            start_x = env.ros.x
            start_y = env.ros.y

            prev_x = start_x
            prev_y = start_y
            traveled = 0.0
            total_reward = 0.0

            terminated = False
            truncated = False
            final_info = {}

            while not (terminated or truncated):
                action, _ = model.predict(
                    obs,
                    deterministic=True,
                )

                obs, reward, terminated, truncated, final_info = env.step(
                    action
                )

                total_reward += float(reward)

                x = env.ros.x
                y = env.ros.y

                traveled += math.hypot(
                    x - prev_x,
                    y - prev_y,
                )

                prev_x = x
                prev_y = y

            success = bool(final_info.get("goal_reached", False))
            collision = bool(final_info.get("collision", False))
            timeout = bool(truncated and not terminated)

            if success:
                successes += 1
                result = "SUCCESS"
            elif collision:
                collisions += 1
                result = "COLLISION"
            elif timeout:
                timeouts += 1
                result = "TIMEOUT"
            else:
                result = "OTHER"

            steps = int(env.episode_steps)
            path_len = float(
                final_info.get(
                    "global_path_length",
                    info["global_path_length"],
                )
            )

            goal_error = float(
                final_info.get(
                    "global_goal_distance",
                    999.0,
                )
            )

            clearance = float(
                final_info.get(
                    "min_clearance",
                    999.0,
                )
            )

            episode_steps.append(steps)
            episode_rewards.append(total_reward)
            actual_distances.append(traveled)
            global_paths.append(path_len)
            final_goal_errors.append(goal_error)
            clearances.append(clearance)

            efficiency = (
                path_len / traveled
                if traveled > 1e-6
                else 0.0
            )

            print(
                f"[{ep + 1:02d}/{episodes:02d}] "
                f"{result:9s} | "
                f"goal=({final_info.get('global_goal_x', 0):5.2f},"
                f"{final_info.get('global_goal_y', 0):5.2f}) | "
                f"steps={steps:4d} | "
                f"path={path_len:5.2f}m | "
                f"travel={traveled:6.2f}m | "
                f"eff={efficiency:5.2f} | "
                f"goal_err={goal_error:5.2f}m | "
                f"clr={clearance:4.2f}m | "
                f"reward={total_reward:7.2f}"
            )

            time.sleep(0.2)

        success_rate = 100.0 * successes / episodes
        collision_rate = 100.0 * collisions / episodes
        timeout_rate = 100.0 * timeouts / episodes

        mean_steps = np.mean(episode_steps)
        mean_reward = np.mean(episode_rewards)
        mean_travel = np.mean(actual_distances)
        mean_path = np.mean(global_paths)
        mean_goal_error = np.mean(final_goal_errors)
        mean_clearance = np.mean(clearances)

        successful_efficiencies = [
            p / d
            for p, d, ok in zip(
                global_paths,
                actual_distances,
                [False] * episodes,
            )
            if d > 1e-6
        ]

        print("\n" + "=" * 78)
        print("FINAL EVALUATION")
        print("=" * 78)

        print(f"Success rate      : {success_rate:6.2f}%")
        print(f"Collision rate    : {collision_rate:6.2f}%")
        print(f"Timeout rate      : {timeout_rate:6.2f}%")
        print(f"Mean steps        : {mean_steps:6.1f}")
        print(f"Mean reward       : {mean_reward:7.2f}")
        print(f"Mean planner path : {mean_path:6.2f} m")
        print(f"Mean travel       : {mean_travel:6.2f} m")
        print(f"Mean goal error   : {mean_goal_error:6.2f} m")
        print(f"Mean clearance    : {mean_clearance:6.2f} m")

        print("=" * 78)

    finally:
        env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "model",
        help="Path to SAC .zip model",
    )

    parser.add_argument(
        "--episodes",
        type=int,
        default=6,
    )

    args = parser.parse_args()

    run_evaluation(
        args.model,
        args.episodes,
    )
