#!/usr/bin/env python3
"""
Evaluate per-frame pallet detection error against Gazebo ground truth.

Subscribes:
  - /pallet_detector/pallet_pose  (PoseStamped, in robot_frame=base_link)
  - /gazebo/model_states          (ground truth pallet world pose)

Looks up base_link <- world via tf2 to compare in a common frame, then
records: stamp, range_to_pallet, x/y/z error, yaw error.

On shutdown prints count + RMSE/mean/std and (optionally) writes CSV.
"""
import math
import csv
import os
from collections import deque

import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs  # registers PoseStamped transform
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates


# Convention rotation that takes gt's local axes (X = pallet long axis,
# Y = width, Z = up) to the detector's published convention (X = horizontal
# width along front face, Y = vertical up, Z = outward face normal).
# Equivalent to a 120 deg rotation about (1,-1,-1)/sqrt(3) in the local frame.
# Pre-computed once so we can apply it via a quaternion multiply per frame.
_R_CONV_QUAT = (0.5, -0.5, -0.5, 0.5)  # (x, y, z, w)


def yaw_from_quat(q):
    return _yaw_from_quat_xyzw(q.x, q.y, q.z, q.w)


def _yaw_from_quat_xyzw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class PalletPoseEvaluator:
    def __init__(self):
        self.gt_model = rospy.get_param("~gt_model", "pallet")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.csv_path = rospy.get_param("~csv_path", "")
        self.match_window = rospy.Duration(rospy.get_param("~match_window", 0.5))
        # Long-axis length of the pallet model. Used to shift gt origin from
        # the geometric centre to the front face centre (where the detector publishes).
        self.pallet_length = rospy.get_param("~pallet_length", 1.22)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)

        self.gt_history = deque(maxlen=300)  # (stamp, x, y, yaw)
        self.records = []  # (stamp, range, ex, ey, ez, eyaw_deg)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.gt_cb, queue_size=1)
        rospy.Subscriber("/pallet_detector/pallet_pose", PoseStamped, self.det_cb,
                         queue_size=10)

        rospy.on_shutdown(self.summarize)
        rospy.loginfo("evaluator: comparing /pallet_detector/pallet_pose to gazebo model '%s'",
                      self.gt_model)

    def gt_cb(self, msg: ModelStates):
        try:
            i = msg.name.index(self.gt_model)
        except ValueError:
            return
        p = msg.pose[i]
        self.gt_history.append((rospy.Time.now(),
                                p.position.x, p.position.y, p.position.z,
                                yaw_from_quat(p.orientation)))

    def nearest_gt(self, stamp):
        if not self.gt_history:
            return None
        # /gazebo/model_states has no stamp; assume "current". Just take the
        # latest sample within the match window of the detection stamp.
        for s, x, y, z, yaw in reversed(self.gt_history):
            if abs((s - stamp).to_sec()) <= self.match_window.to_sec():
                return x, y, z, yaw
        return self.gt_history[-1][1:]  # fall back to most recent

    def det_cb(self, msg: PoseStamped):
        try:
            pose_world = self.tf_buf.transform(msg, self.world_frame,
                                               timeout=rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            rospy.logwarn_throttle(2.0, "tf transform failed: %s", e)
            return

        gt = self.nearest_gt(msg.header.stamp)
        if gt is None:
            return
        gt_x, gt_y, gt_z, gt_yaw = gt

        # Bring gt into the detector's published convention before comparing.
        # 1) Origin: detector publishes at the front face centre, gt is at the
        #    pallet geometric centre. Shift along gt's local -X (long axis) by L/2.
        half_L = 0.5 * self.pallet_length
        gt_face_x = gt_x - half_L * math.cos(gt_yaw)
        gt_face_y = gt_y - half_L * math.sin(gt_yaw)
        gt_face_z = gt_z
        # 2) Orientation: re-label gt's local axes (X=long, Y=width, Z=up) to the
        #    detector's (X=width, Y=up, Z=outward face normal) by right-multiplying
        #    by the convention quaternion in the local frame.
        gt_quat = (0.0, 0.0, math.sin(gt_yaw * 0.5), math.cos(gt_yaw * 0.5))
        gt_face_quat = tft.quaternion_multiply(gt_quat, _R_CONV_QUAT)
        gt_face_yaw = _yaw_from_quat_xyzw(*gt_face_quat)

        dx = pose_world.pose.position.x - gt_face_x
        dy = pose_world.pose.position.y - gt_face_y
        dz = pose_world.pose.position.z - gt_face_z
        det_yaw = yaw_from_quat(pose_world.pose.orientation)
        eyaw_deg = math.degrees(wrap_pi(det_yaw - gt_face_yaw))

        rng = math.hypot(gt_x - 0.0, gt_y - 0.0)  # from world origin; replace if needed
        self.records.append((msg.header.stamp.to_sec(), rng, dx, dy, dz, eyaw_deg))

        rospy.loginfo("frame@%.2fs  err: dx=%+.3f dy=%+.3f dz=%+.3f  yaw=%+.2fdeg",
                      msg.header.stamp.to_sec(), dx, dy, dz, eyaw_deg)

    def summarize(self):
        n = len(self.records)
        if n == 0:
            rospy.logwarn("evaluator: no matched frames")
            return

        arr = np.array([r[2:] for r in self.records])  # dx, dy, dz, eyaw
        rmse = np.sqrt((arr ** 2).mean(axis=0))
        mean = arr.mean(axis=0)
        std = arr.std(axis=0)
        pos_err = np.linalg.norm(arr[:, :2], axis=1)

        msg = (
            "\n========= PALLET POSE EVAL ({} frames) =========\n"
            "  dx  : mean={:+.4f}  std={:.4f}  rmse={:.4f} m\n"
            "  dy  : mean={:+.4f}  std={:.4f}  rmse={:.4f} m\n"
            "  dxy : mean={:.4f}   std={:.4f}  rmse={:.4f} m\n"
            "  yaw : mean={:+.3f}  std={:.3f}  rmse={:.3f} deg\n"
            "  |yaw>5deg| frames: {} / {} ({:.1f}%)\n"
            "================================================"
        ).format(
            n,
            mean[0], std[0], rmse[0],
            mean[1], std[1], rmse[1],
            pos_err.mean(), pos_err.std(), np.sqrt((pos_err ** 2).mean()),
            mean[3], std[3], rmse[3],
            int((np.abs(arr[:, 3]) > 5.0).sum()), n,
            100.0 * (np.abs(arr[:, 3]) > 5.0).mean(),
        )
        rospy.loginfo(msg)

        if self.csv_path:
            path = os.path.expanduser(self.csv_path)
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["stamp", "range_m", "dx_m", "dy_m", "dz_m", "yaw_err_deg"])
                w.writerows(self.records)
            rospy.loginfo("evaluator: wrote %d rows to %s", n, path)


if __name__ == "__main__":
    rospy.init_node("pallet_pose_evaluator")
    PalletPoseEvaluator()
    rospy.spin()
