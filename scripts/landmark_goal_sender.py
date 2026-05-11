#!/usr/bin/env python3
"""
landmark_goal_sender.py

Reads a QR landmark from landmarks.yaml and sends its approach_pose to Nav2.

Example:
    python3 landmark_goal_sender.py DOCK
    python3 landmark_goal_sender.py RESTAURANT --landmarks-file /home/hadi/amr_project/param/landmarks.yaml
    python3 landmark_goal_sender.py --list
"""

import argparse
import math
import os
import sys
from typing import Any, Dict

import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose


DEFAULT_LANDMARKS_FILE = "/home/hadi/amr_project/landmarks.yaml"


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw


def load_landmarks(path: str) -> Dict[str, Any]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"landmarks file does not exist: {path}")
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"invalid landmarks YAML format in {path}")
    return data


class LandmarkGoalSender(Node):
    def __init__(self, use_sim_time: bool):
        super().__init__("landmark_goal_sender")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, use_sim_time)])
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def send_goal(self, label: str, entry: Dict[str, Any], use_qr_pose: bool = False) -> bool:
        frame = entry.get("reference_frame", "")
        if frame != "map":
            self.get_logger().error(
                f"Landmark '{label}' is stored in reference_frame='{frame}', not 'map'. "
                "Do not use old odom landmarks after restarting with a saved map. "
                "Re-run QR localization during mapping and save the landmark in map frame."
            )
            return False

        pose_key = "qr_pose" if use_qr_pose else "approach_pose"
        if pose_key not in entry:
            self.get_logger().error(f"Landmark '{label}' has no '{pose_key}' field.")
            return False

        pose = entry[pose_key]
        try:
            x = float(pose["x"])
            y = float(pose["y"])
            yaw = float(pose.get("yaw", 0.0))
        except Exception as exc:
            self.get_logger().error(f"Invalid pose for landmark '{label}': {exc}")
            return False

        self.get_logger().info("Waiting for Nav2 NavigateToPose action server...")
        if not self.client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("Nav2 NavigateToPose action server is not available.")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Sending Nav2 goal for '{label}' using {pose_key}: "
            f"x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg"
        )

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by Nav2.")
            return False

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            self.get_logger().error("No result received from Nav2.")
            return False

        status = result.status
        if status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info(f"Reached landmark '{label}'.")
            return True

        self.get_logger().warn(f"Navigation ended with status code {status}.")
        return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("landmark", nargs="?", help="Landmark name, for example DOCK or RESTAURANT")
    parser.add_argument("--landmarks-file", default=DEFAULT_LANDMARKS_FILE)
    parser.add_argument("--use-qr-pose", action="store_true", help="Navigate to the QR pose itself instead of the safer approach_pose")
    parser.add_argument("--list", action="store_true", help="List available landmarks and exit")
    parser.add_argument("--use-sim-time", default="true", choices=("true", "false"))
    args = parser.parse_args()

    try:
        landmarks = load_landmarks(args.landmarks_file)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if args.list:
        if not landmarks:
            print("No landmarks found.")
            return 0
        print("Available landmarks:")
        for name in sorted(landmarks.keys()):
            frame = landmarks[name].get("reference_frame", "unknown")
            pose = landmarks[name].get("approach_pose", {})
            x = pose.get("x", "?")
            y = pose.get("y", "?")
            print(f"  {name:20s} frame={frame:5s} approach=({x}, {y})")
        return 0

    if not args.landmark:
        print("ERROR: give a landmark name or use --list", file=sys.stderr)
        return 2

    label = args.landmark.strip()
    if label not in landmarks:
        print(f"ERROR: landmark '{label}' not found.", file=sys.stderr)
        print("Available landmarks:", ", ".join(sorted(landmarks.keys())), file=sys.stderr)
        return 2

    rclpy.init()
    node = LandmarkGoalSender(use_sim_time=(args.use_sim_time == "true"))
    try:
        ok = node.send_goal(label, landmarks[label], use_qr_pose=args.use_qr_pose)
        return 0 if ok else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
