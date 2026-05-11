#!/usr/bin/env python3
"""
qr_localizer.py — detect QR codes, estimate their position in the world using
camera bearing + lidar range (with visual-range cross-check), and save approach
poses to ~/amr/landmarks.yaml so the robot can navigate back to them later.

Pipeline per frame:
  1. Decode QR codes with pyzbar; get image-plane bounding box.
  2. Pixel center -> bearing (using camera intrinsics from /camera/camera_info).
  3. Estimate range visually from bbox height + known QR physical size.
  4. Transform bearing from camera frame to lidar frame via TF.
  5. Read /scan at that bearing -> lidar range.
  6. Cross-check: if |lidar_range - visual_range| > VISUAL_RANGE_TOLERANCE,
     the lidar ray is hitting the wrong object. Prefer visual range.
  7. Compute QR position in reference frame (map preferred, odom fallback).
  8. Compute a 1.5 m approach pose facing the QR.
  9. Fuse with previous sightings (quality-weighted running mean); persist.

Run (after bridge + Gazebo are up):
    python3 ~/amr/qr_localizer.py
"""

import math
import os
import time
from datetime import datetime, timezone

import numpy as np
import cv2
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode as zbar_decode

import tf2_ros
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # registers do_transform_point on PointStamped


# -------------------- tunables --------------------
APPROACH_DISTANCE_M      = 1.5    # how far in front of the QR the goal pose sits
MIN_BBOX_HEIGHT_PX       = 25     # reject QRs smaller than this (too far / too small)
MAX_CENTER_OFFSET_FRAC   = 0.8    # reject QRs outside +/- 80% of image width
LIDAR_MIN_RANGE_M        = 0.2
LIDAR_MAX_RANGE_M        = 15.0
LIDAR_AVG_WINDOW_DEG     = 0.8    # tight window: QR sign is only 4cm thick
MAX_DRIFT_NEW_SIGHTING_M = 2.5    # reject sightings further than this from stored

# Effective decodable QR size (not the full sign!). The sign board is 0.96m
# in the SDF, but the QR image has a white quiet zone around the pattern, so
# pyzbar's bounding box only spans the dark-module region — empirically ~0.64m.
# If you ever need to recalibrate: compare visual range to lidar range in a
# clean head-on sighting, and set this to 0.96 * (lidar_range / visual_range).
QR_PHYSICAL_SIZE_M       = 0.64

# NEW: visual-vs-lidar range sanity check
VISUAL_RANGE_TOLERANCE_FRAC = 0.30  # lidar must be within +/-30% of visual estimate
MAX_VISUAL_RANGE_M          = 12.0  # reject estimates further than this

# Novelty gate
MIN_MOVEMENT_M           = 0.10
MIN_ROTATION_DEG         = 3.0

# Log throttling
LOG_MIN_DELTA_M          = 0.05

LANDMARKS_FILE = os.path.expanduser("~/amr_project/param/landmarks.yaml")

ROBOT_BASE_FRAME = "chassis"
LIDAR_FRAME      = "lidar_frame"
PREFERRED_REF_FRAMES = ["map", "odom"]

# Multi-camera config. Each camera has an independent image+info stream and
# its own TF frame; fusion still keys on the QR label so detections from
# any camera collapse into one landmark entry.
CAMERAS = [
    {"name": "front", "frame": "camera_frame",       "image_topic": "/camera/image_raw",       "info_topic": "/camera/camera_info"},
    {"name": "left",  "frame": "camera_left_frame",  "image_topic": "/camera_left/image_raw",  "info_topic": "/camera_left/camera_info"},
    {"name": "right", "frame": "camera_right_frame", "image_topic": "/camera_right/image_raw", "info_topic": "/camera_right/camera_info"},
    {"name": "back",  "frame": "camera_back_frame",  "image_topic": "/camera_back/image_raw",  "info_topic": "/camera_back/camera_info"},
]
CAMERAS_BY_NAME = {c["name"]: c for c in CAMERAS}

# Set >1 to skip frames per camera and reduce CPU under load.
PROCESS_EVERY_N_FRAMES = 1


def yaw_from_quat(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff_deg(a_deg, b_deg):
    d = (a_deg - b_deg + 180.0) % 360.0 - 180.0
    return abs(d)


class QRLocalizer(Node):
    def __init__(self):
        super().__init__("qr_localizer")

        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_infos = {}                                # name -> CameraInfo
        self.frames_seen  = {c["name"]: 0 for c in CAMERAS}   # name -> int
        self.latest_scan = None

        self.landmarks = self._load_landmarks()
        # Keyed on (camera_name, label) so different camera angles of the same
        # QR each independently update fusion instead of suppressing each other.
        self._last_sighting_robot_pose = {}

        for cfg in CAMERAS:
            name = cfg["name"]
            # n=name default-arg pins the loop variable to avoid Python late-binding;
            # without it, all three callbacks would close over the final value of `name`.
            self.create_subscription(
                CameraInfo, cfg["info_topic"],
                lambda msg, n=name: self._on_camera_info(n, msg), qos_sensor)
            self.create_subscription(
                Image, cfg["image_topic"],
                lambda msg, n=name: self._on_image(n, msg), qos_sensor)

        self.create_subscription(LaserScan, "/scan",
                                 self._on_scan, qos_sensor)

        self._last_warning_time = 0.0
        self.active_ref_frame = None

        # Async-flush timer: writes landmarks.yaml at most 1 Hz when dirty,
        # so synchronous yaml.safe_dump never blocks the image callback.
        self._landmarks_dirty = False
        self.create_timer(1.0, self._flush_landmarks_to_disk)

        self.get_logger().info(
            f"QR localizer up. Saving to {LANDMARKS_FILE}. "
            f"Waiting for camera_info + /scan + image frames..."
        )
        if self.landmarks:
            self.get_logger().info(
                f"Loaded {len(self.landmarks)} existing landmark(s): "
                f"{sorted(self.landmarks.keys())}"
            )

    # -------------------- persistence --------------------
    def _load_landmarks(self):
        if not os.path.exists(LANDMARKS_FILE):
            return {}
        try:
            with open(LANDMARKS_FILE, "r") as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().warn(f"Could not load {LANDMARKS_FILE}: {e}")
            return {}

    def _save_landmarks(self):
        # Synchronous landmarks.yaml writes inside the image callback used to
        # block the executor for 10-50ms per QR sighting, which backed up
        # /scan callbacks and made SLAM drop scans (visible as fragmented
        # map). Now this just flags dirty; a 1Hz timer does the actual write.
        self._landmarks_dirty = True

    def _flush_landmarks_to_disk(self):
        """Timer callback: write yaml at most once per second if anything changed."""
        if not getattr(self, "_landmarks_dirty", False):
            return
        self._landmarks_dirty = False
        try:
            clean = self._pythonify(self.landmarks)
            with open(LANDMARKS_FILE, "w") as f:
                yaml.safe_dump(clean, f, sort_keys=True,
                               default_flow_style=False)
        except Exception as e:
            self.get_logger().warn(f"Could not save {LANDMARKS_FILE}: {e}")

    @staticmethod
    def _pythonify(obj):
        """Recursively convert numpy scalars to Python floats/ints."""
        if isinstance(obj, dict):
            return {k: QRLocalizer._pythonify(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [QRLocalizer._pythonify(v) for v in obj]
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.integer):
            return int(obj)
        return obj

    # -------------------- callbacks --------------------
    def _on_camera_info(self, name, msg: CameraInfo):
        self.camera_infos[name] = msg

    def _on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def _on_image(self, name, msg: Image):
        if name not in self.camera_infos or self.latest_scan is None:
            return

        self.frames_seen[name] += 1
        if PROCESS_EVERY_N_FRAMES > 1 and (self.frames_seen[name] % PROCESS_EVERY_N_FRAMES) != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed ({name}): {e}")
            return

        stamp = msg.header.stamp
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = zbar_decode(gray)

        for r in results:
            text = r.data.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            self._draw_detection(frame, r, text)
            self._process_sighting(name, frame, r, text, stamp)

        cv2.putText(frame, f"ref: {self.active_ref_frame or '(no tf yet)'}  cam: {CAMERAS_BY_NAME[name]['frame']}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, f"landmarks: {len(self.landmarks)}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.imshow(f"QR localizer - {name}", frame)
        cv2.waitKey(1)

    # -------------------- detection drawing --------------------
    def _draw_detection(self, frame, r, text):
        pts = [(p.x, p.y) for p in r.polygon]
        if len(pts) >= 3:
            poly = np.array(pts, dtype=int).reshape(-1, 1, 2)
            cv2.polylines(frame, [poly], True, (0, 255, 0), 2)
        cv2.putText(frame, text, (r.rect.left, max(r.rect.top - 8, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # -------------------- main sighting logic --------------------
    def _process_sighting(self, name, frame, r, label, stamp):
        bbox_h = r.rect.height
        bbox_w = r.rect.width
        if bbox_h < MIN_BBOX_HEIGHT_PX:
            return

        u = r.rect.left + bbox_w / 2.0
        v = r.rect.top + bbox_h / 2.0

        info = self.camera_infos[name]
        camera_frame = CAMERAS_BY_NAME[name]["frame"]
        cx = info.k[2]
        cy = info.k[5]
        fx = info.k[0]
        fy = info.k[4]

        if abs(u - cx) > MAX_CENTER_OFFSET_FRAC * cx:
            return

        # --- Step 1: bearing in camera frame ---
        bearing_cam = math.atan2(u - cx, fx)

        # --- Step 2: visual range estimate from bbox size ---
        # Use the larger of width/height; at steep viewing angles the width
        # shortens first (foreshortening), so height is the more robust axis.
        # Pick max in case the camera roll ever changes.
        apparent_px = max(bbox_h, bbox_w)
        visual_range = (QR_PHYSICAL_SIZE_M * fy) / apparent_px

        if visual_range > MAX_VISUAL_RANGE_M:
            return

        # --- Step 3: bearing in lidar frame ---
        ray_cam = np.array([math.cos(bearing_cam),
                            -math.sin(bearing_cam),
                            0.0])
        try:
            tf_cam_to_lidar = self.tf_buffer.lookup_transform(
                LIDAR_FRAME, camera_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except tf2_ros.TransformException as e:
            self._throttle_warn(f"TF {camera_frame}->{LIDAR_FRAME}: {e}")
            return
        ray_lidar = self._rotate_vec_by_transform(ray_cam, tf_cam_to_lidar)
        bearing_lidar = math.atan2(ray_lidar[1], ray_lidar[0])

        # --- Step 4: lidar range at that bearing ---
        scan = self.latest_scan
        window_rad = math.radians(LIDAR_AVG_WINDOW_DEG)
        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

        bearing_norm = math.atan2(math.sin(bearing_lidar), math.cos(bearing_lidar))
        mask = np.abs(((angles - bearing_norm + math.pi) % (2 * math.pi)) - math.pi) < window_rad
        window = ranges[mask]
        window = window[np.isfinite(window)]
        window = window[(window >= LIDAR_MIN_RANGE_M) & (window <= LIDAR_MAX_RANGE_M)]

        # --- Step 5: choose best range with cross-check ---
        # Prefer lidar when it agrees with the visual estimate (lidar is more
        # accurate at distance). Fall back to visual when lidar disagrees or
        # returns nothing (sign is too thin for the ray to hit).
        range_source = None
        range_m = None

        if window.size > 0:
            lidar_range_candidate = float(np.min(window))  # nearest hit in window
            err_frac = abs(lidar_range_candidate - visual_range) / max(visual_range, 0.5)
            if err_frac <= VISUAL_RANGE_TOLERANCE_FRAC:
                range_m = lidar_range_candidate
                range_source = "lidar"
            else:
                range_m = visual_range
                range_source = f"visual(lidar_said_{lidar_range_candidate:.1f})"
        else:
            range_m = visual_range
            range_source = "visual(no_lidar)"

        # --- Step 6: point in reference frame ---
        qr_in_lidar = np.array([
            range_m * math.cos(bearing_lidar),
            range_m * math.sin(bearing_lidar),
            0.0,
        ])

        ref_frame, qr_point_ref = self._point_to_reference_frame(
            qr_in_lidar, LIDAR_FRAME, stamp)
        if ref_frame is None:
            return
        self.active_ref_frame = ref_frame

        try:
            tf_base_in_ref = self.tf_buffer.lookup_transform(
                ref_frame, ROBOT_BASE_FRAME, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except tf2_ros.TransformException as e:
            self._throttle_warn(f"TF {ref_frame}->{ROBOT_BASE_FRAME}: {e}")
            return

        rx = tf_base_in_ref.transform.translation.x
        ry = tf_base_in_ref.transform.translation.y
        ryaw = yaw_from_quat(
            tf_base_in_ref.transform.rotation.x,
            tf_base_in_ref.transform.rotation.y,
            tf_base_in_ref.transform.rotation.z,
            tf_base_in_ref.transform.rotation.w,
        )

        # --- novelty gate (per-camera, per-label) ---
        novelty_key = (name, label)
        last = self._last_sighting_robot_pose.get(novelty_key)
        if last is not None:
            lx, ly, lyaw = last
            moved = math.hypot(rx - lx, ry - ly)
            rotated_deg = angle_diff_deg(math.degrees(ryaw), math.degrees(lyaw))
            if moved < MIN_MOVEMENT_M and rotated_deg < MIN_ROTATION_DEG:
                return

        # --- approach pose ---
        qx, qy = float(qr_point_ref[0]), float(qr_point_ref[1])
        dx = rx - qx
        dy = ry - qy
        dist_robot_to_qr = math.hypot(dx, dy)
        if dist_robot_to_qr < 0.2:
            return

        ux = dx / dist_robot_to_qr
        uy = dy / dist_robot_to_qr
        approach_x = qx + ux * APPROACH_DISTANCE_M
        approach_y = qy + uy * APPROACH_DISTANCE_M
        approach_yaw = math.atan2(qy - approach_y, qx - approach_x)

        # --- quality score ---
        size_score   = min(1.0, bbox_h / 120.0)
        center_score = 1.0 - abs(u - cx) / (MAX_CENTER_OFFSET_FRAC * cx)
        range_score  = max(0.0, 1.0 - range_m / LIDAR_MAX_RANGE_M)
        # Visual-only sightings are a touch less trustworthy than lidar-confirmed ones
        source_score = 0.7 if range_source.startswith("visual") else 1.0
        quality = float(np.clip(size_score * center_score * range_score * source_score,
                                1e-3, 1.0))

        self._last_sighting_robot_pose[novelty_key] = (rx, ry, ryaw)

        self._fuse_and_save(label, qx, qy,
                            approach_x, approach_y, approach_yaw,
                            quality, range_m, ref_frame, range_source,
                            visual_range)

    # -------------------- fusion + persistence --------------------
    def _fuse_and_save(self, label, qx, qy, ax, ay, ayaw, quality,
                       range_m, ref_frame, range_source, visual_range):
        now_iso = datetime.now(timezone.utc).isoformat(timespec="seconds")

        existing = self.landmarks.get(label)
        if existing is None:
            self.landmarks[label] = {
                "qr_pose":       {"x": float(qx), "y": float(qy)},
                "approach_pose": {"x": float(ax), "y": float(ay),
                                  "yaw": float(ayaw)},
                "sightings": 1,
                "total_weight": quality,
                "best_quality": quality,
                "last_range_m": range_m,
                "reference_frame": ref_frame,
                "last_updated": now_iso,
            }
            self.get_logger().info(
                f"[{label}] first sighting: QR=({qx:.2f}, {qy:.2f}) "
                f"approach=({ax:.2f}, {ay:.2f}, yaw={math.degrees(ayaw):.0f}deg) "
                f"q={quality:.2f} range={range_m:.2f}m ({range_source}) "
                f"vis={visual_range:.2f}m frame={ref_frame}"
            )
            self._save_landmarks()
            return

        prev_qx = existing["qr_pose"]["x"]
        prev_qy = existing["qr_pose"]["y"]
        drift = math.hypot(qx - prev_qx, qy - prev_qy)
        if drift > MAX_DRIFT_NEW_SIGHTING_M and existing["sightings"] >= 3:
            self.get_logger().warn(
                f"[{label}] rejecting: drift {drift:.1f}m from stored "
                f"(range_source={range_source}, visual_range={visual_range:.2f}m)"
            )
            return

        if existing.get("reference_frame") != ref_frame:
            self.get_logger().info(
                f"[{label}] reference frame {existing.get('reference_frame')} "
                f"-> {ref_frame}, replacing"
            )
            self.landmarks[label] = {
                "qr_pose":       {"x": float(qx), "y": float(qy)},
                "approach_pose": {"x": float(ax), "y": float(ay),
                                  "yaw": float(ayaw)},
                "sightings": 1,
                "total_weight": quality,
                "best_quality": quality,
                "last_range_m": range_m,
                "reference_frame": ref_frame,
                "last_updated": now_iso,
            }
            self._save_landmarks()
            return

        prev_total_w = existing.get("total_weight", 1.0)
        new_total_w = prev_total_w + quality
        w_new = quality / new_total_w

        new_qx = (1 - w_new) * prev_qx + w_new * qx
        new_qy = (1 - w_new) * prev_qy + w_new * qy

        prev_ax = existing["approach_pose"]["x"]
        prev_ay = existing["approach_pose"]["y"]
        prev_ayaw = existing["approach_pose"]["yaw"]
        new_ax = (1 - w_new) * prev_ax + w_new * ax
        new_ay = (1 - w_new) * prev_ay + w_new * ay
        vx = (1 - w_new) * math.cos(prev_ayaw) + w_new * math.cos(ayaw)
        vy = (1 - w_new) * math.sin(prev_ayaw) + w_new * math.sin(ayaw)
        new_ayaw = math.atan2(vy, vx)

        estimate_shift = math.hypot(new_qx - prev_qx, new_qy - prev_qy)

        existing["qr_pose"]       = {"x": float(new_qx), "y": float(new_qy)}
        existing["approach_pose"] = {"x": float(new_ax), "y": float(new_ay),
                                     "yaw": float(new_ayaw)}
        existing["sightings"] += 1
        existing["total_weight"] = float(new_total_w)
        existing["best_quality"] = float(max(existing.get("best_quality", 0.0),
                                             quality))
        existing["last_range_m"] = range_m
        existing["last_updated"] = now_iso

        if estimate_shift >= LOG_MIN_DELTA_M:
            self.get_logger().info(
                f"[{label}] #{existing['sightings']}: "
                f"QR=({new_qx:.2f}, {new_qy:.2f}) [shift {estimate_shift*100:.0f}cm] "
                f"approach=({new_ax:.2f}, {new_ay:.2f}, "
                f"yaw={math.degrees(new_ayaw):.0f}deg) "
                f"q={quality:.2f} best={existing['best_quality']:.2f} "
                f"range={range_m:.2f}m ({range_source})"
            )

        self._save_landmarks()

    # -------------------- TF helpers --------------------
    def _point_to_reference_frame(self, xyz_in_lidar, source_frame, stamp):
        for target in PREFERRED_REF_FRAMES:
            try:
                p = PointStamped()
                p.header.stamp = rclpy.time.Time().to_msg()
                p.header.frame_id = source_frame
                p.point.x = float(xyz_in_lidar[0])
                p.point.y = float(xyz_in_lidar[1])
                p.point.z = float(xyz_in_lidar[2])
                out = self.tf_buffer.transform(
                    p, target,
                    timeout=rclpy.duration.Duration(seconds=0.1))
                return target, np.array([out.point.x, out.point.y, out.point.z])
            except Exception:
                continue
        self._throttle_warn(
            f"Could not transform from {source_frame} to any of "
            f"{PREFERRED_REF_FRAMES}"
        )
        return None, None

    def _rotate_vec_by_transform(self, vec, tf_msg):
        qx = tf_msg.transform.rotation.x
        qy = tf_msg.transform.rotation.y
        qz = tf_msg.transform.rotation.z
        qw = tf_msg.transform.rotation.w
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        R = np.array([
            [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
            [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)],
        ])
        return R @ vec

    def _throttle_warn(self, msg, period_s=2.0):
        now = time.time()
        if now - self._last_warning_time > period_s:
            self.get_logger().warn(msg)
            self._last_warning_time = now


def main():
    rclpy.init()
    node = QRLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
