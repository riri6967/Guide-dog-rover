#!/usr/bin/env python3
import os
from pathlib import Path
from collections import deque

import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import (
    BaseCallback,
    CallbackList,
    CheckpointCallback,
)
from stable_baselines3.common.monitor import Monitor

from assistive_sac_env_v9 import (
    AssistiveNavEnvV9,
    LidarCNNExtractor,
)


class EpisodeMetricsCallback(BaseCallback):
    """Log rolling episode outcomes to TensorBoard."""

    def __init__(self, window=20, verbose=0):
        super().__init__(verbose)
        self.success = deque(maxlen=window)
        self.collisions = deque(maxlen=window)
        self.timeouts = deque(maxlen=window)

    def _on_step(self):
        dones = self.locals.get("dones")
        infos = self.locals.get("infos")

        if dones is None or infos is None:
            return True

        for done, info in zip(dones, infos):
            if not done:
                continue

            episode = info.get("episode")
            if episode is None:
                continue

            if "goal_reached" in episode:
                self.success.append(
                    float(episode["goal_reached"])
                )
            if "collision" in episode:
                self.collisions.append(
                    float(episode["collision"])
                )
            if "sensor_timeout" in episode:
                self.timeouts.append(
                    float(episode["sensor_timeout"])
                )

        if self.success:
            self.logger.record(
                "rollout/success_rate_20ep",
                100.0 * sum(self.success) / len(self.success),
            )
        if self.collisions:
            self.logger.record(
                "rollout/collision_rate_20ep",
                100.0 * sum(self.collisions) / len(self.collisions),
            )
        if self.timeouts:
            self.logger.record(
                "rollout/sensor_timeout_rate_20ep",
                100.0 * sum(self.timeouts) / len(self.timeouts),
            )

        return True


def main():
    base = Path.home() / "assistive_robot_ws" / "models" / "sac_v9"
    base.mkdir(parents=True, exist_ok=True)

    print("=" * 72)
    print("V9 SAC TRAINING")
    print("=" * 72)
    print("Policy: LiDAR CNN + local subgoal")
    print("Observation: 727")
    print("Action: 2")
    print("CUDA:", torch.cuda.is_available())
    if torch.cuda.is_available():
        print("GPU:", torch.cuda.get_device_name(0))

    env = Monitor(
        AssistiveNavEnvV9(
            max_episode_steps=2500,
            goal_lookahead=1.5,
        ),
        info_keywords=(
            "goal_reached",
            "collision",
            "sensor_timeout",
        ),
    )

    policy_kwargs = dict(
        features_extractor_class=LidarCNNExtractor,
        features_extractor_kwargs=dict(
            features_dim=256,
        ),
        net_arch=dict(
            pi=[256, 256],
            qf=[256, 256],
        ),
    )

    model = SAC(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        buffer_size=100_000,
        learning_starts=int(os.environ.get("V9_LEARNING_STARTS", "2000")),
        batch_size=256,
        tau=0.005,
        gamma=0.99,
        train_freq=1,
        gradient_steps=1,
        ent_coef="auto",
        seed=int(os.environ.get("V9_SEED", "42")),
        policy_kwargs=policy_kwargs,
        verbose=1,
        tensorboard_log=str(base / "tb"),
        device="cuda" if torch.cuda.is_available() else "cpu",
    )

    checkpoint = CheckpointCallback(
        save_freq=10_000,
        save_path=str(base / "checkpoints"),
        name_prefix="sac_v9",
        save_replay_buffer=True,
        save_vecnormalize=False,
    )

    episode_metrics = EpisodeMetricsCallback(window=20)
    callbacks = CallbackList([checkpoint, episode_metrics])

    total_timesteps = int(
        os.environ.get("V9_STEPS", "50000")
    )

    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=callbacks,
            progress_bar=True,
        )
        model.save(str(base / "assistive_sac_v9_final"))
        env.close()
        print("=" * 72)
        print("V9 TRAINING SAVED")
        print(base / "assistive_sac_v9_final")
        print("=" * 72)
    except KeyboardInterrupt:
        emergency = base / "assistive_sac_v9_interrupt"
        model.save(str(emergency))
        print("\n[V9] Ctrl+C caught. Emergency checkpoint saved:")
        print(emergency)
        try:
            env.close()
        except Exception:
            pass
    except Exception:
        emergency = base / "assistive_sac_v9_crash"
        model.save(str(emergency))
        print("\n[V9] Crash checkpoint saved:")
        print(emergency)
        try:
            env.close()
        except Exception:
            pass
        raise


if __name__ == "__main__":
    main()
