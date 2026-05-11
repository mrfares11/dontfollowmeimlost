#!/usr/bin/env python3
"""Bumper-driven escape node for service_robot.

Subscribes to /bumper/{front,rear,left,right}.  When any reports physical
contact lasting > 0.2s, takes over /cmd_vel for 1.5s with a directional
escape twist (back away from the contact, optionally rotate).  Then enters
1.0s cooldown so Nav2 sees the new pose, plans a new path, resumes.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts


DEBOUNCE_S    = 0.2     # contact must persist this long
ESCAPE_DUR_S  = 1.5     # length of escape maneuver
COOLDOWN_S    = 1.0     # silent gap after escape before re-arming
PUB_HZ        = 30.0    # publish rate during escape

V_BACK   = -0.4
V_FWD    =  0.4
V_DIAG   = -0.3
V_SIDE   = -0.25
W_TURN   =  0.6
W_SPIN   =  0.7


class BumperEscape(Node):
    def __init__(self):
        super().__init__('bumper_escape')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.contact = {'front': False, 'rear': False, 'left': False, 'right': False}
        self.contact_since = {k: None for k in self.contact}

        for side in self.contact:
            self.create_subscription(
                Contacts, f'/bumper/{side}',
                lambda msg, s=side: self._cb(s, msg),
                sensor_qos,
            )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.state = 'IDLE'
        self.state_start = None
        self.escape_twist = Twist()

        self.create_timer(1.0 / PUB_HZ, self._tick)
        self.get_logger().info('Bumper escape node armed.')

    def _cb(self, side, msg):
        active = len(msg.contacts) > 0
        now = self.get_clock().now()
        if active:
            if not self.contact[side]:
                self.contact_since[side] = now
            self.contact[side] = True
        else:
            self.contact[side] = False
            self.contact_since[side] = None

    def _stable_contacts(self):
        """Return set of bumpers currently stably triggered (past debounce)."""
        now = self.get_clock().now()
        result = set()
        for side, since in self.contact_since.items():
            if since is None:
                continue
            if (now - since).nanoseconds / 1e9 >= DEBOUNCE_S:
                result.add(side)
        return result

    def _decide_twist(self, hits):
        t = Twist()
        if hits == {'front'}:
            t.linear.x, t.angular.z = V_BACK, 0.0
        elif hits == {'rear'}:
            t.linear.x, t.angular.z = V_FWD, 0.0
        elif hits == {'left'}:
            t.linear.x, t.angular.z = V_SIDE, -W_TURN
        elif hits == {'right'}:
            t.linear.x, t.angular.z = V_SIDE,  W_TURN
        elif hits == {'front', 'left'}:
            t.linear.x, t.angular.z = V_DIAG, -W_TURN
        elif hits == {'front', 'right'}:
            t.linear.x, t.angular.z = V_DIAG,  W_TURN
        elif hits == {'rear', 'left'}:
            t.linear.x, t.angular.z =  V_DIAG, -W_TURN
        elif hits == {'rear', 'right'}:
            t.linear.x, t.angular.z =  V_DIAG,  W_TURN
        else:
            t.linear.x, t.angular.z = V_DIAG,  W_SPIN
        return t

    def _tick(self):
        now = self.get_clock().now()

        if self.state == 'IDLE':
            hits = self._stable_contacts()
            if hits:
                self.escape_twist = self._decide_twist(hits)
                self.state = 'ESCAPING'
                self.state_start = now
                self.get_logger().warn(
                    f'BUMPER HIT {sorted(hits)} -> escape '
                    f'lin={self.escape_twist.linear.x:.2f} '
                    f'ang={self.escape_twist.angular.z:.2f}')
            return

        elapsed = (now - self.state_start).nanoseconds / 1e9

        if self.state == 'ESCAPING':
            if elapsed < ESCAPE_DUR_S:
                self.cmd_pub.publish(self.escape_twist)
            else:
                self.cmd_pub.publish(Twist())  # one explicit stop
                self.state = 'COOLDOWN'
                self.state_start = now
                self.get_logger().info('Escape complete -> cooldown')
            return

        if self.state == 'COOLDOWN':
            if elapsed >= COOLDOWN_S:
                self.state = 'IDLE'
                self.get_logger().info('Re-armed.')


def main():
    rclpy.init()
    rclpy.spin(BumperEscape())


if __name__ == '__main__':
    main()
