#!/usr/bin/env python3
"""
qr_scanner.py — subscribe to /camera/image_raw, detect QR codes with pyzbar,
print the decoded text once per unique value, and show a live OpenCV window
with green boxes + labels on detected codes.

Run (after bridge + Gazebo are up):
    python3 ~/amr/qr_scanner.py

Dependencies:
    sudo apt install ros-humble-cv-bridge python3-opencv libzbar0
    pip3 install pyzbar
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from pyzbar.pyzbar import decode as zbar_decode


class QRScanner(Node):
    def __init__(self):
        super().__init__("qr_scanner")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.bridge = CvBridge()
        self.seen = set()

        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.on_image, qos
        )

        self.get_logger().info(
            "QR scanner up (pyzbar). Waiting for frames on /camera/image_raw ..."
        )

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        # pyzbar is faster + more forgiving on grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = zbar_decode(gray)

        for r in results:
            text = r.data.decode("utf-8", errors="replace")
            if not text:
                continue

            # Draw bounding polygon
            pts = [(p.x, p.y) for p in r.polygon]
            if len(pts) >= 3:
                poly = np.array(pts, dtype=int).reshape(-1, 1, 2)
                cv2.polylines(frame, [poly], True, (0, 255, 0), 2)

            # Draw label
            x, y = r.rect.left, r.rect.top
            cv2.putText(
                frame, text, (x, max(y - 8, 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
            )

            if text not in self.seen:
                self.seen.add(text)
                self.get_logger().info(f"NEW QR: {text}")
                self.get_logger().info(
                    f"   (seen so far: {sorted(self.seen)})"
                )

        cv2.imshow("Robot camera - QR scanner", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = QRScanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        # Guard against double-shutdown (fixes the traceback you saw on Ctrl+C)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
