#!/usr/bin/env python3
"""
coverage_tour.py — drive the robot to a grid of waypoints across the saved
OccupancyGrid (rotating in place at each) so any QR signs missed during
frontier exploration get a chance to be seen by the front/left/right cameras.

Runs after explore_and_save_map.py finishes. Subscribes to /map, generates a
collision-free grid of waypoints, orders them with nearest-neighbor from the
current robot pose, and visits them via Nav2's navigate_to_pose action. At
each waypoint, optionally rotates 360 (or configured arc) in place by
publishing directly to /cmd_vel.
"""

import argparse
import math
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy,
)

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import tf2_ros
from tf2_ros import Buffer, TransformListener

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:
    sys.stderr.write(
        "coverage_tour.py requires scipy. Install it with:\n"
        "    sudo apt install python3-scipy\n"
    )
    sys.exit(2)


def parse_bool(s):
    return str(s).strip().lower() in ("true", "1", "yes", "y", "on")


def yaw_to_quat(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def yaw_from_quat(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class CoverageTour(Node):
    def __init__(self):
        super().__init__("coverage_tour")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # /map is published with TRANSIENT_LOCAL durability by slam_toolbox/nav2,
        # so we match that to receive the latched message even if we connect late.
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_msg = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self._shutdown = False

    def _on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def request_shutdown(self):
        self._shutdown = True

    # -------------------- waypoint generation --------------------
    def wait_for_map(self, timeout_s=30.0):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not self._shutdown and self.map_msg is None:
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.5)
        return self.map_msg is not None

    def generate_waypoints(self, grid_spacing_m, min_clearance_m):
        msg = self.map_msg
        h, w = msg.info.height, msg.info.width
        res = msg.info.resolution
        x0 = msg.info.origin.position.x
        y0 = msg.info.origin.position.y

        # ROS OccupancyGrid convention: row 0 corresponds to y = y_min (origin.y),
        # and y INCREASES with row index. We do NOT flip rows even if a debug
        # imshow looks "upside down" — that's just image-display convention vs.
        # map-frame convention. (r, c) -> (x, y) is computed without row flip.
        data = np.array(msg.data, dtype=np.int16).reshape((h, w))

        # Treat unknown (-1) AND occupied (100) as obstacles for safety so we
        # don't try to drive into unexplored cells.
        obstacle = (data != 0)

        dist_cells = distance_transform_edt(~obstacle)
        dist_m = dist_cells * res

        stride = max(1, int(round(grid_spacing_m / res)))
        candidates = []
        for r in range(stride // 2, h, stride):
            for c in range(stride // 2, w, stride):
                if dist_m[r, c] >= min_clearance_m:
                    x = x0 + (c + 0.5) * res
                    y = y0 + (r + 0.5) * res
                    candidates.append((float(x), float(y)))

        self.get_logger().info(
            f"[coverage_tour] map: {w}x{h} cells, res={res:.2f}m, "
            f"origin=({x0:.2f}, {y0:.2f})"
        )
        self.get_logger().info(
            f"[coverage_tour] generated {len(candidates)} waypoints "
            f"(grid={grid_spacing_m}m, clearance={min_clearance_m}m)"
        )
        return candidates

    # -------------------- robot pose --------------------
    def get_robot_pose_in_map(self, retry_s=5.0):
        deadline = time.monotonic() + retry_s
        while rclpy.ok() and not self._shutdown and time.monotonic() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    "map", "chassis", rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                yaw = yaw_from_quat(
                    tf.transform.rotation.x, tf.transform.rotation.y,
                    tf.transform.rotation.z, tf.transform.rotation.w)
                return float(x), float(y), float(yaw)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rclpy.spin_once(self, timeout_sec=0.2)
        return None

    @staticmethod
    def order_nearest_neighbor(start_xy, candidates):
        if not candidates:
            return []
        remaining = list(candidates)
        ordered = []
        cur = start_xy
        while remaining:
            idx = min(range(len(remaining)),
                      key=lambda i: (remaining[i][0] - cur[0]) ** 2
                                  + (remaining[i][1] - cur[1]) ** 2)
            cur = remaining.pop(idx)
            ordered.append(cur)
        return ordered

    # -------------------- nav2 goal --------------------
    def send_goal_and_wait(self, x, y, yaw, timeout_s):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return "rejected", 0.0

        start = time.monotonic()
        result_future = goal_handle.get_result_async()
        deadline = start + timeout_s
        while rclpy.ok() and not self._shutdown and not result_future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
                return "timeout", time.monotonic() - start
            rclpy.spin_once(self, timeout_sec=min(0.5, remaining))

        elapsed = time.monotonic() - start
        if self._shutdown:
            try:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            except Exception:
                pass
            return "canceled", elapsed

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return "succeeded", elapsed
        return "failed", elapsed

    # -------------------- in-place rotation --------------------
    def rotate_in_place(self, degrees, speed_rad_s):
        if degrees == 0 or speed_rad_s <= 0:
            return
        duration = abs(math.radians(degrees) / speed_rad_s)
        sign = 1.0 if degrees >= 0 else -1.0
        twist = Twist()
        twist.angular.z = sign * abs(speed_rad_s)
        period = 0.1
        n_iters = max(1, int(round(duration / period)))
        for _ in range(n_iters):
            if not rclpy.ok() or self._shutdown:
                break
            self.cmd_vel_pub.publish(twist)
            time.sleep(period)
        self.cmd_vel_pub.publish(Twist())  # explicit zero stop

    # -------------------- main loop --------------------
    def run(self, args):
        if not self.wait_for_map(timeout_s=30.0):
            self.get_logger().error("[coverage_tour] no /map received within 30s; aborting")
            return 0

        pose = self.get_robot_pose_in_map(retry_s=5.0)
        if pose is None:
            self.get_logger().error("[coverage_tour] could not look up map->chassis TF; aborting")
            return 0
        rx, ry, ryaw = pose

        candidates = self.generate_waypoints(args.grid_spacing, args.min_clearance)
        if not candidates:
            self.get_logger().warn("[coverage_tour] no valid waypoints generated; nothing to do")
            return 0

        ordered = self.order_nearest_neighbor((rx, ry), candidates)
        if args.max_waypoints > 0:
            ordered = ordered[:args.max_waypoints]

        if not self.nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("[coverage_tour] navigate_to_pose action server not available; aborting")
            return 0

        rotate = parse_bool(args.rotate_at_waypoints)
        succeeded = 0
        failed = 0
        n = len(ordered)

        for i, (x, y) in enumerate(ordered):
            if self._shutdown:
                break

            if i + 1 < n:
                nx, ny = ordered[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = ryaw

            yaw_deg = math.degrees(yaw)
            self.get_logger().info(
                f"[coverage_tour] {i+1}/{n} -> (x={x:.2f}, y={y:.2f}) yaw={yaw_deg:.0f}deg"
            )
            outcome, elapsed = self.send_goal_and_wait(x, y, yaw, args.goal_timeout)

            if outcome == "succeeded":
                succeeded += 1
                self.get_logger().info(
                    f"[coverage_tour] {i+1}/{n} arrived in {elapsed:.1f}s"
                )
            else:
                failed += 1
                self.get_logger().warn(
                    f"[coverage_tour] {i+1}/{n} {outcome} after {elapsed:.1f}s"
                )

            if rotate and not self._shutdown:
                self.get_logger().info(
                    f"[coverage_tour] {i+1}/{n} rotating {args.rotation_degrees:.0f}deg "
                    f"@ {args.rotation_speed:.2f} rad/s"
                )
                self.rotate_in_place(args.rotation_degrees, args.rotation_speed)

        self.get_logger().info(
            f"[coverage_tour] tour complete: {succeeded}/{n} succeeded, {failed} failed"
        )
        return 0


def build_arg_parser():
    p = argparse.ArgumentParser(description="Drive the robot to a grid of waypoints across the saved map.")
    p.add_argument("--grid-spacing", type=float, default=2.0,
                   help="Target spacing between waypoints in meters.")
    p.add_argument("--min-clearance", type=float, default=0.4,
                   help="Minimum free distance (m) from a waypoint to any obstacle/unknown cell.")
    p.add_argument("--rotate-at-waypoints", default="true", choices=("true", "false"),
                   help="Rotate in place at each waypoint after Nav2 reports done.")
    p.add_argument("--rotation-degrees", type=float, default=180.0,
                   help="Degrees to rotate at each waypoint (sign sets direction).")
    p.add_argument("--rotation-speed", type=float, default=0.5,
                   help="Rotation angular speed in rad/s.")
    p.add_argument("--goal-timeout", type=float, default=60.0,
                   help="Per-waypoint timeout in seconds.")
    p.add_argument("--use-sim-time", default="true", choices=("true", "false"),
                   help="Set the use_sim_time parameter on this node.")
    p.add_argument("--max-waypoints", type=int, default=0,
                   help="Cap the visited waypoints (0 = unlimited).")
    return p


def main():
    args = build_arg_parser().parse_args()

    rclpy.init()
    node = CoverageTour()
    node.set_parameters([
        rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL,
                                  parse_bool(args.use_sim_time)),
    ])

    def _handle_sigint(signum, frame):
        node.get_logger().warn("[coverage_tour] SIGINT received; shutting down")
        node.request_shutdown()

    signal.signal(signal.SIGINT, _handle_sigint)

    rc = 0
    try:
        rc = node.run(args)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_vel_pub.publish(Twist())  # safety: stop the robot
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
