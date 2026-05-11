#!/usr/bin/env python3
"""
landmark_nav_gui.py

Simple GUI for phase 2 navigation.

- Reads /home/hadi/amr_project/landmarks.yaml
- Shows all valid saved landmarks
- Lets you select one landmark
- Sends its approach_pose to Nav2 NavigateToPose when you press Confirm

Run manually:
    cd /home/hadi/amr_project
    python3 landmark_nav_gui.py

The phase2 GUI launch file starts this automatically.
"""

import math
import os
import threading
import tkinter as tk
from tkinter import messagebox

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


DEFAULT_LANDMARKS_FILE = "/home/hadi/amr_project/param/landmarks.yaml"
NAV_ACTION_NAME = "navigate_to_pose"


def yaw_to_quaternion(yaw: float):
    """Return quaternion tuple for yaw-only rotation."""
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qx, qy, qz, qw


class LandmarkNavGui(Node):
    def __init__(self):
        super().__init__("landmark_nav_gui")

        self.declare_parameter("landmarks_file", DEFAULT_LANDMARKS_FILE)
        self.landmarks_file = (
            self.get_parameter("landmarks_file")
            .get_parameter_value()
            .string_value
        )

        self.action_client = ActionClient(self, NavigateToPose, NAV_ACTION_NAME)
        self.landmarks = {}
        self.current_goal_handle = None

        # -------------------- Tkinter GUI --------------------
        self.root = tk.Tk()
        self.root.title("AMR Landmark Navigator")
        self.root.geometry("620x430")

        title = tk.Label(
            self.root,
            text="Select a saved landmark target",
            font=("Arial", 18, "bold"),
        )
        title.pack(pady=(15, 5))

        self.status_var = tk.StringVar()
        self.status_var.set("Loading landmarks...")

        self.status_label = tk.Label(
            self.root,
            textvariable=self.status_var,
            font=("Arial", 11),
            fg="blue",
            wraplength=560,
            justify="center",
        )
        self.status_label.pack(pady=(0, 10))

        main_frame = tk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=20)

        left_frame = tk.Frame(main_frame)
        left_frame.pack(side="left", fill="both", expand=True)

        tk.Label(
            left_frame,
            text="Locations",
            font=("Arial", 12, "bold"),
        ).pack(anchor="w")

        self.listbox = tk.Listbox(
            left_frame,
            font=("Arial", 13),
            height=12,
            exportselection=False,
        )
        self.listbox.pack(fill="both", expand=True, pady=(5, 0))
        self.listbox.bind("<<ListboxSelect>>", self.on_select)

        right_frame = tk.Frame(main_frame)
        right_frame.pack(side="right", fill="both", expand=True, padx=(20, 0))

        tk.Label(
            right_frame,
            text="Selected target",
            font=("Arial", 12, "bold"),
        ).pack(anchor="w")

        self.info_text = tk.Text(
            right_frame,
            height=12,
            font=("Arial", 11),
            wrap="word",
            state="disabled",
        )
        self.info_text.pack(fill="both", expand=True, pady=(5, 0))

        button_frame = tk.Frame(self.root)
        button_frame.pack(fill="x", padx=20, pady=15)

        self.refresh_button = tk.Button(
            button_frame,
            text="Refresh landmarks",
            font=("Arial", 12),
            command=self.refresh_landmarks,
        )
        self.refresh_button.pack(side="left")

        self.confirm_button = tk.Button(
            button_frame,
            text="Confirm and Navigate",
            font=("Arial", 12, "bold"),
            bg="#4CAF50",
            fg="white",
            command=self.confirm_navigation,
        )
        self.confirm_button.pack(side="right")

        self.cancel_button = tk.Button(
            button_frame,
            text="Cancel current goal",
            font=("Arial", 12),
            command=self.cancel_goal,
        )
        self.cancel_button.pack(side="right", padx=(0, 10))

        self.refresh_landmarks()

        # Spin ROS periodically without blocking Tkinter.
        self.root.after(100, self.spin_ros_once)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # -------------------- landmark loading --------------------
    def refresh_landmarks(self):
        self.landmarks = self.load_landmarks(self.landmarks_file)
        self.listbox.delete(0, tk.END)

        valid_names = sorted(self.landmarks.keys())
        for name in valid_names:
            self.listbox.insert(tk.END, name)

        if valid_names:
            self.status_var.set(
                f"Loaded {len(valid_names)} valid map-frame landmark(s) from {self.landmarks_file}"
            )
        else:
            self.status_var.set(
                f"No valid map-frame landmarks found in {self.landmarks_file}"
            )

        self.show_info(None)

    def load_landmarks(self, path):
        if not os.path.exists(path):
            self.get_logger().warn(f"Landmarks file does not exist: {path}")
            return {}

        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(f"Could not read landmarks file: {exc}")
            return {}

        valid = {}
        for name, entry in data.items():
            if not isinstance(entry, dict):
                continue

            ref = entry.get("reference_frame", "")
            pose = entry.get("approach_pose", None)

            if ref != "map":
                self.get_logger().warn(
                    f"Skipping {name}: reference_frame is '{ref}', expected 'map'"
                )
                continue

            if not isinstance(pose, dict):
                self.get_logger().warn(f"Skipping {name}: missing approach_pose")
                continue

            if not all(k in pose for k in ("x", "y", "yaw")):
                self.get_logger().warn(
                    f"Skipping {name}: approach_pose must contain x, y, yaw"
                )
                continue

            try:
                valid[name] = {
                    "x": float(pose["x"]),
                    "y": float(pose["y"]),
                    "yaw": float(pose["yaw"]),
                    "sightings": int(entry.get("sightings", 0)),
                    "best_quality": float(entry.get("best_quality", 0.0)),
                    "last_updated": str(entry.get("last_updated", "")),
                }
            except Exception as exc:
                self.get_logger().warn(f"Skipping {name}: invalid pose values: {exc}")

        return valid

    # -------------------- GUI events --------------------
    def selected_name(self):
        selection = self.listbox.curselection()
        if not selection:
            return None
        return self.listbox.get(selection[0])

    def on_select(self, _event=None):
        self.show_info(self.selected_name())

    def show_info(self, name):
        self.info_text.configure(state="normal")
        self.info_text.delete("1.0", tk.END)

        if name is None:
            self.info_text.insert(tk.END, "No target selected.\n")
        else:
            lm = self.landmarks[name]
            self.info_text.insert(tk.END, f"Name: {name}\n\n")
            self.info_text.insert(tk.END, "Nav2 goal frame: map\n")
            self.info_text.insert(tk.END, f"x:   {lm['x']:.3f} m\n")
            self.info_text.insert(tk.END, f"y:   {lm['y']:.3f} m\n")
            self.info_text.insert(
                tk.END,
                f"yaw: {math.degrees(lm['yaw']):.1f} deg\n\n",
            )
            self.info_text.insert(tk.END, f"sightings: {lm['sightings']}\n")
            self.info_text.insert(tk.END, f"best quality: {lm['best_quality']:.3f}\n")
            if lm["last_updated"]:
                self.info_text.insert(tk.END, f"last updated: {lm['last_updated']}\n")

        self.info_text.configure(state="disabled")

    def confirm_navigation(self):
        name = self.selected_name()
        if name is None:
            messagebox.showwarning("No target selected", "Select a location first.")
            return

        if name not in self.landmarks:
            messagebox.showerror("Invalid target", f"Target '{name}' was not found.")
            return

        if not self.action_client.wait_for_server(timeout_sec=1.0):
            messagebox.showerror(
                "Nav2 not ready",
                "NavigateToPose action server is not available yet.\n"
                "Wait until Nav2 is fully active, then try again.",
            )
            return

        lm = self.landmarks[name]
        result = messagebox.askyesno(
            "Confirm navigation",
            (
                f"Send robot to {name}?\n\n"
                f"x = {lm['x']:.2f} m\n"
                f"y = {lm['y']:.2f} m\n"
                f"yaw = {math.degrees(lm['yaw']):.1f} deg"
            ),
        )
        if not result:
            return

        self.send_goal(name, lm)

    # -------------------- Nav2 action --------------------
    def send_goal(self, name, lm):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = lm["x"]
        goal_msg.pose.pose.position.y = lm["y"]
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(lm["yaw"])
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.status_var.set(f"Sending goal to {name}...")
        self.get_logger().info(
            f"Sending Nav2 goal for {name}: "
            f"x={lm['x']:.3f}, y={lm['y']:.3f}, yaw={math.degrees(lm['yaw']):.1f} deg"
        )

        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        future.add_done_callback(lambda fut: self.goal_response_callback(fut, name))

    def goal_response_callback(self, future, name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.status_var.set(f"Goal rejected by Nav2: {name}")
            self.get_logger().error(f"Goal rejected by Nav2: {name}")
            messagebox.showerror(
                "Goal rejected",
                f"Nav2 rejected the goal for {name}.\n"
                "Check localization, map->chassis TF, and whether the target is reachable.",
            )
            return

        self.current_goal_handle = goal_handle
        self.status_var.set(f"Goal accepted: navigating to {name}")
        self.get_logger().info(f"Goal accepted: {name}")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self.result_callback(fut, name))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        try:
            dist = feedback.distance_remaining
            self.status_var.set(f"Navigating... distance remaining: {dist:.2f} m")
        except Exception:
            self.status_var.set("Navigating...")

    def result_callback(self, future, name):
        result = future.result()
        status = result.status

        # action_msgs/msg/GoalStatus:
        # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if status == 4:
            self.status_var.set(f"Arrived at {name}")
            self.get_logger().info(f"Navigation succeeded: {name}")
            messagebox.showinfo("Navigation complete", f"Arrived at {name}.")
        elif status == 5:
            self.status_var.set(f"Goal canceled: {name}")
            self.get_logger().warn(f"Navigation canceled: {name}")
        else:
            self.status_var.set(f"Navigation failed/aborted: {name}")
            self.get_logger().error(f"Navigation failed/aborted: {name}, status={status}")

        self.current_goal_handle = None

    def cancel_goal(self):
        if self.current_goal_handle is None:
            self.status_var.set("No active goal to cancel.")
            return

        self.status_var.set("Canceling current goal...")
        future = self.current_goal_handle.cancel_goal_async()
        future.add_done_callback(lambda _fut: self.status_var.set("Cancel request sent."))

    # -------------------- ROS/Tkinter loop --------------------
    def spin_ros_once(self):
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception as exc:
            self.get_logger().error(f"spin_once error: {exc}")
        self.root.after(50, self.spin_ros_once)

    def on_close(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = LandmarkNavGui()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
