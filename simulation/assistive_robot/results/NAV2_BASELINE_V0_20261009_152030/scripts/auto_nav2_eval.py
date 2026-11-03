#!/usr/bin/env python3

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from ros_gz_interfaces.msg import Contacts


PROJECT = Path(__file__).resolve().parents[1]


def path_length(path):
    poses = path.poses
    if len(poses) < 2:
        return None

    total = 0.0
    for a, b in zip(poses[:-1], poses[1:]):
        dx = b.pose.position.x - a.pose.position.x
        dy = b.pose.position.y - a.pose.position.y
        total += math.hypot(dx, dy)

    return total


class Nav2AutoEvaluator(Node):
    def __init__(self, collision_topic=None, near_threshold=0.65):
        super().__init__("nav2_auto_evaluator")

        self.action_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )

        self.create_subscription(
            Odometry, "/odom", self.odom_cb, 10
        )
        self.create_subscription(
            LaserScan, "/scan", self.scan_cb, 10
        )
        self.create_subscription(
            NavPath, "/plan", self.plan_cb, 10
        )

        self.collision_topic = collision_topic
        if collision_topic:
            self.create_subscription(
                Contacts, collision_topic, self.collision_cb, 10
            )

        self.near_threshold = near_threshold

        self.latest_position = None
        self.last_distance_position = None
        self.actual_distance = 0.0

        self.recording = False
        self.plan_sequence = 0
        self.plan_sequence_at_start = 0
        self.planned_path_length = None

        self.min_lidar_range = None
        self.near_obstacle_samples = 0

        self.contact_reported = False
        self.contact_samples = 0
        self.contact_pairs = set()

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        current = (float(p.x), float(p.y))

        if self.recording and self.last_distance_position is not None:
            dx = current[0] - self.last_distance_position[0]
            dy = current[1] - self.last_distance_position[1]
            self.actual_distance += math.hypot(dx, dy)

        self.latest_position = current
        self.last_distance_position = current

    def scan_cb(self, msg):
        if not msg.ranges:
            return

        valid = []
        for r in msg.ranges:
            if math.isnan(r):
                continue
            if math.isinf(r):
                r = msg.range_max if r > 0 else msg.range_min
            valid.append(float(r))

        if not valid:
            return

        current_min = min(valid)

        if self.recording:
            if self.min_lidar_range is None:
                self.min_lidar_range = current_min
            else:
                self.min_lidar_range = min(
                    self.min_lidar_range, current_min
                )

            if current_min < self.near_threshold:
                self.near_obstacle_samples += 1

    def plan_cb(self, msg):
        self.plan_sequence += 1

        # Only use a path published after this goal began.
        if (
            self.recording
            and self.plan_sequence > self.plan_sequence_at_start
        ):
            length = path_length(msg)
            if length is not None:
                self.planned_path_length = length

    def collision_cb(self, msg):
        if not self.recording or not msg.contacts:
            return

        self.contact_reported = True
        self.contact_samples += 1

        for item in msg.contacts:
            first = item.collision1.name or str(item.collision1.id)
            second = item.collision2.name or str(item.collision2.id)
            self.contact_pairs.add(f"{first} <-> {second}")

    def begin_measurement(self):
        self.actual_distance = 0.0
        self.last_distance_position = self.latest_position
        self.plan_sequence_at_start = self.plan_sequence
        self.planned_path_length = None
        self.min_lidar_range = None
        self.near_obstacle_samples = 0
        self.contact_reported = False
        self.contact_samples = 0
        self.contact_pairs = set()
        self.recording = True

    def finish_measurement(self):
        self.recording = False

    def execute_goal(self, goal, timeout_sec):
        name = goal["name"]
        gx = float(goal["x"])
        gy = float(goal["y"])
        yaw = math.radians(float(goal.get("yaw_deg") or 0.0))
        frame = goal.get("frame") or "map"

        self.begin_measurement()
        started = time.monotonic()

        request = NavigateToPose.Goal()
        request.pose = PoseStamped()
        request.pose.header.frame_id = frame
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x = gx
        request.pose.pose.position.y = gy
        request.pose.pose.position.z = 0.0
        request.pose.pose.orientation.z = math.sin(yaw / 2.0)
        request.pose.pose.orientation.w = math.cos(yaw / 2.0)

        print(f"\n[{name}] Sending goal: ({gx:.2f}, {gy:.2f}) frame={frame}")

        send_future = self.action_client.send_goal_async(request)
        send_deadline = time.monotonic() + 10.0

        while rclpy.ok() and not send_future.done():
            if time.monotonic() >= send_deadline:
                self.finish_measurement()
                return self.make_row(
                    goal, "GOAL_RESPONSE_TIMEOUT", False, True,
                    time.monotonic() - started
                )
            rclpy.spin_once(self, timeout_sec=0.05)

        if not send_future.done():
            self.finish_measurement()
            return self.make_row(
                goal, "GOAL_RESPONSE_TIMEOUT", False, True,
                time.monotonic() - started
            )

        handle = send_future.result()

        if handle is None or not handle.accepted:
            self.finish_measurement()
            return self.make_row(
                goal, "REJECTED", False, False,
                time.monotonic() - started
            )

        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout_sec
        timed_out = False
        cancel_unconfirmed = False

        while rclpy.ok() and not result_future.done():
            if time.monotonic() >= deadline:
                timed_out = True
                print(f"[{name}] Timeout; requesting goal cancellation.")
                cancel_future = handle.cancel_goal_async()

                cancel_deadline = time.monotonic() + 5.0
                while (
                    rclpy.ok()
                    and not cancel_future.done()
                    and time.monotonic() < cancel_deadline
                ):
                    rclpy.spin_once(self, timeout_sec=0.05)

                # Wait briefly for Nav2 to finish cancellation before
                # attempting the next goal.
                result_deadline = time.monotonic() + 5.0
                while (
                    rclpy.ok()
                    and not result_future.done()
                    and time.monotonic() < result_deadline
                ):
                    rclpy.spin_once(self, timeout_sec=0.05)

                cancel_unconfirmed = not result_future.done()
                break

            rclpy.spin_once(self, timeout_sec=0.05)

        elapsed = time.monotonic() - started

        if timed_out:
            status = (
                "TIMEOUT_CANCEL_UNCONFIRMED"
                if cancel_unconfirmed
                else "TIMEOUT"
            )
            success = False
        elif result_future.done():
            result = result_future.result()
            code = result.status
            status_names = {
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_ABORTED: "ABORTED",
                GoalStatus.STATUS_CANCELED: "CANCELED",
            }
            status = status_names.get(code, f"STATUS_{code}")
            success = code == GoalStatus.STATUS_SUCCEEDED
        else:
            status = "INTERRUPTED"
            success = False

        self.finish_measurement()

        row = self.make_row(
            goal, status, success, timed_out, elapsed
        )

        print(
            f"[{name}] {status} | success={int(success)} | "
            f"time={elapsed:.1f}s | "
            f"distance={self.actual_distance:.2f}m | "
            f"planned={row['planned_path_length_m']} | "
            f"near-obstacle samples={self.near_obstacle_samples}"
        )

        return row

    def make_row(self, goal, status, success, timeout, elapsed):
        planned = self.planned_path_length
        actual = self.actual_distance

        if planned is not None and actual > 1e-3:
            efficiency = planned / actual
        else:
            efficiency = ""

        final_goal_distance = ""
        if self.latest_position is not None:
            final_goal_distance = math.hypot(
                self.latest_position[0] - float(goal["x"]),
                self.latest_position[1] - float(goal["y"]),
            )

        return {
            "timestamp": datetime.now().astimezone().isoformat(),
            "goal_name": goal["name"],
            "frame": goal.get("frame") or "map",
            "goal_x": goal["x"],
            "goal_y": goal["y"],
            "status": status,
            "success": int(success),
            "timeout": int(timeout),
            "duration_s": round(elapsed, 3),
            "actual_distance_m": round(self.actual_distance, 3),
            "planned_path_length_m": (
                round(planned, 3) if planned is not None else ""
            ),
            "route_efficiency_ratio": (
                round(efficiency, 4) if efficiency != "" else ""
            ),
            "final_goal_distance_m": (
                round(final_goal_distance, 3)
                if final_goal_distance != "" else ""
            ),
            "min_lidar_range_m": (
                round(self.min_lidar_range, 3)
                if self.min_lidar_range is not None else ""
            ),
            "near_obstacle_samples": self.near_obstacle_samples,
            # Blank means no confirmed contact-sensor source was configured.
            "contact_reported": (
                int(self.contact_reported) if self.collision_topic else ""
            ),
            "contact_samples": (
                self.contact_samples if self.collision_topic else ""
            ),
            "contact_pairs": (
                "; ".join(sorted(self.contact_pairs))
                if self.collision_topic else ""
            ),
            "collision_data_available": int(
                bool(self.collision_topic)
            ),
            "collision_topic": self.collision_topic or "",
        }


def load_goals(path):
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        raise ValueError(f"No goals found in {path}")

    for i, row in enumerate(rows, start=1):
        if not row.get("name"):
            row["name"] = f"goal_{i}"
        if "x" not in row or "y" not in row:
            raise ValueError("Goals CSV must contain x and y columns.")
        float(row["x"])
        float(row["y"])
        float(row.get("yaw_deg") or 0.0)

    return rows


def main():
    parser = argparse.ArgumentParser(
        description="Automatically send Nav2 goals and write CSV metrics."
    )
    parser.add_argument(
        "--goals-csv",
        type=Path,
        default=PROJECT / "nav2" / "goals.csv",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=120.0,
        help="Maximum seconds per goal",
    )
    parser.add_argument(
        "--pause-between",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--near-threshold",
        type=float,
        default=0.65,
        help="LiDAR range below which a scan is counted as near-obstacle",
    )
    parser.add_argument(
        "--collision-topic",
        default="/rover_contacts",
        help="Gazebo ros_gz_interfaces/Contacts topic",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
    )
    args = parser.parse_args()

    goals = load_goals(args.goals_csv)
    output = args.output or (
        PROJECT / "results"
        / f"nav2_eval_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )
    output.parent.mkdir(parents=True, exist_ok=True)

    fields = [
        "timestamp", "goal_name", "frame", "goal_x", "goal_y",
        "status", "success", "timeout", "duration_s",
        "actual_distance_m", "planned_path_length_m",
        "route_efficiency_ratio", "final_goal_distance_m",
        "min_lidar_range_m", "near_obstacle_samples",
        "contact_reported", "contact_samples", "contact_pairs",
        "collision_data_available", "collision_topic",
    ]

    rclpy.init()
    node = Nav2AutoEvaluator(
        collision_topic=args.collision_topic,
        near_threshold=args.near_threshold,
    )

    try:
        if not node.action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                "NavigateToPose action server unavailable. "
                "Check that Nav2 is active."
            )

        print(f"Goals: {len(goals)}")
        print(f"CSV output: {output}")
        print("RViz can remain open during the test.")
        if not args.collision_topic:
            print(
                "WARNING: physical collisions will be UNKNOWN in the CSV "
                "until a valid contact Bool topic is configured. "
                "LiDAR near-obstacle samples are only a proxy."
            )

        with output.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            f.flush()

            for i, goal in enumerate(goals):
                row = node.execute_goal(goal, args.timeout)
                writer.writerow(row)
                f.flush()

                print(f"CSV updated: {output}")

                if row["status"] == "TIMEOUT_CANCEL_UNCONFIRMED":
                    print(
                        "Stopping the campaign: Nav2 goal cancellation "
                        "could not be confirmed. Check Nav2 before continuing."
                    )
                    break

                if i < len(goals) - 1:
                    end = time.monotonic() + args.pause_between
                    while rclpy.ok() and time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=0.05)

        print(f"\nEvaluation finished. Results: {output}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
