#!/usr/bin/env python3
"""
Patrol controller for Phase 2 dynamic obstacles.

Reads patrol routes from param/patrol_routes.yaml and drives each patrol
diff-drive bot back-and-forth between waypoints. Each patrol publishes
geometry_msgs/Twist (linear.x + angular.z) to its cmd_vel topic
(e.g. /patrol_1/cmd_vel) which is bridged into Gazebo's DiffDrive plugin.

Motion model: non-holonomic turn-then-drive. If heading error exceeds
YAW_THRESHOLD_RAD, the bot turns in place; otherwise it drives forward
with a small angular correction.

Position + heading are tracked by dead-reckoning from commanded velocity
(no odom subscription). Each patrol's initial_position and initial_yaw
must match the corresponding SDF <pose> so dead-reckoning starts in sync.
Tolerance is loose (0.5 m) so accumulated drift is acceptable.
"""

import os
import math
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


DEFAULT_YAML_PATH = "/home/hadi/amr_project/param/patrol_routes.yaml"
PUBLISH_HZ = 10.0

# Turn-then-drive thresholds
YAW_THRESHOLD_RAD       = 0.20   # turn in place if heading error > ~11.5 deg
MAX_ANGULAR_TURNING     = 0.8    # rad/s when turning in place
MAX_ANGULAR_DRIVING     = 0.5    # rad/s heading correction while moving
KP_HEADING              = 1.5    # P gain on heading error


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Patrol:
    """One bidirectional patrol — tracks state and computes next velocity."""

    def __init__(self, name, config, publisher, get_clock):
        self.name = name
        self.cmd_topic = config["cmd_topic"]
        self.waypoints = [(wp["x"], wp["y"]) for wp in config["waypoints"]]
        self.linear_speed = float(config.get("linear_speed", 0.3))
        self.pause_s = float(config.get("pause_at_endpoint_s", 2.0))
        self.tolerance = float(config.get("waypoint_reach_tolerance_m", 0.5))

        init = config.get("initial_position", {"x": self.waypoints[0][0], "y": self.waypoints[0][1]})
        self.x = float(init["x"])
        self.y = float(init["y"])
        self.yaw = float(config.get("initial_yaw", 0.0))

        self.publisher = publisher
        self.get_clock = get_clock

        # State
        self.current_target_idx = 0   # which waypoint we're heading toward
        self.direction = 1            # +1 = forward through list, -1 = reverse
        self.state = "DRIVING"        # DRIVING or PAUSING
        self.pause_started = None

        self.last_tick = self.get_clock().now()

    def tick(self):
        """Called PUBLISH_HZ times per second. Computes and publishes Twist."""
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds / 1e9
        self.last_tick = now
        # Cap dt to avoid huge integration steps if the scheduler hiccups
        if dt > 0.5:
            dt = 0.0

        target = self.waypoints[self.current_target_idx]
        dx = target[0] - self.x
        dy = target[1] - self.y
        dist = math.hypot(dx, dy)

        twist = Twist()

        if self.state == "PAUSING":
            elapsed = (now - self.pause_started).nanoseconds / 1e9
            if elapsed >= self.pause_s:
                self._advance_waypoint()
                self.state = "DRIVING"
            self.publisher.publish(twist)  # zero
            return

        # state == DRIVING
        if dist <= self.tolerance:
            self.state = "PAUSING"
            self.pause_started = now
            self.publisher.publish(twist)  # zero
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = wrap_pi(target_yaw - self.yaw)

        if abs(yaw_err) > YAW_THRESHOLD_RAD:
            # Turn in place — no forward motion until heading is close
            ang = max(-MAX_ANGULAR_TURNING,
                      min(MAX_ANGULAR_TURNING, KP_HEADING * yaw_err))
            twist.angular.z = ang
            self.yaw = wrap_pi(self.yaw + ang * dt)
        else:
            # Drive forward with small heading correction
            ang = max(-MAX_ANGULAR_DRIVING,
                      min(MAX_ANGULAR_DRIVING, KP_HEADING * yaw_err))
            twist.linear.x = self.linear_speed
            twist.angular.z = ang
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self.x += twist.linear.x * c * dt
            self.y += twist.linear.x * s * dt
            self.yaw = wrap_pi(self.yaw + ang * dt)

        self.publisher.publish(twist)

    def _advance_waypoint(self):
        """Move to next waypoint in the bidirectional loop."""
        new_idx = self.current_target_idx + self.direction
        if new_idx >= len(self.waypoints):
            self.direction = -1
            new_idx = self.current_target_idx + self.direction
        elif new_idx < 0:
            self.direction = 1
            new_idx = self.current_target_idx + self.direction
        self.current_target_idx = new_idx


class PatrolController(Node):
    def __init__(self):
        super().__init__("patrol_controller")

        yaml_path = self.declare_parameter(
            "patrol_routes_yaml", DEFAULT_YAML_PATH
        ).value

        if not os.path.isfile(yaml_path):
            self.get_logger().error(
                f"Patrol routes file not found: {yaml_path}"
            )
            raise SystemExit(1)

        with open(yaml_path, "r") as f:
            config = yaml.safe_load(f)

        if "patrols" not in config or not isinstance(config["patrols"], dict):
            self.get_logger().error(
                f"Invalid YAML structure (missing 'patrols' dict): {yaml_path}"
            )
            raise SystemExit(1)

        self.patrols = []
        for name, patrol_cfg in config["patrols"].items():
            cmd_topic = patrol_cfg.get("cmd_topic")
            if not cmd_topic:
                self.get_logger().warn(
                    f"Patrol {name} has no cmd_topic, skipping"
                )
                continue
            pub = self.create_publisher(Twist, cmd_topic, 10)
            patrol = Patrol(name, patrol_cfg, pub, self.get_clock)
            self.patrols.append(patrol)
            self.get_logger().info(
                f"Patrol {name} configured: topic={cmd_topic}, "
                f"waypoints={patrol.waypoints}, speed={patrol.linear_speed}, "
                f"initial yaw={patrol.yaw:.3f} rad"
            )

        if not self.patrols:
            self.get_logger().error("No patrols configured, exiting")
            raise SystemExit(1)

        period_s = 1.0 / PUBLISH_HZ
        self.create_timer(period_s, self._tick_all)
        self.get_logger().info(
            f"Patrol controller running — {len(self.patrols)} patrols, "
            f"publish rate {PUBLISH_HZ} Hz"
        )

    def _tick_all(self):
        for p in self.patrols:
            p.tick()


def main():
    rclpy.init()
    node = PatrolController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
