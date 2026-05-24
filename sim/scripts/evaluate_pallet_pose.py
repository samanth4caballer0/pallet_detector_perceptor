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


# Convention rotation that takes gt's local axes (X = pallet long axis (back),
# Y = width, Z = up) to the detector's published convention (X = forward /
# outward face normal, Y = lateral / pallet face width, Z = up).
# This is a 180 deg rotation about the local Z axis: gt_x and gt_y flip sign,
# gt_z is unchanged. Pre-computed once so we can apply it via quaternion multiply.
# Previous convention (X=width, Y=up, Z=outward) — replaced 2026-05-23:
# _R_CONV_QUAT = (0.5, -0.5, -0.5, 0.5)  # (x, y, z, w)
_R_CONV_QUAT = (0.0, 0.0, 1.0, 0.0)  # (x, y, z, w) = R_z(180 deg)


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
        # Robot model name in gazebo (for world pose) and the URDF root frame
        # used as the bridge between optical and world. The launch file's
        # world_to_base TF is a static identity, so we cannot trust the
        # optical->world chain; we compose to world ourselves using the
        # robot's gazebo pose.
        self.robot_model = rospy.get_param("~robot_model", "duna")
        self.robot_frame = rospy.get_param("~robot_frame", "base")
        self.csv_path = rospy.get_param("~csv_path", "")
        self.match_window = rospy.Duration(rospy.get_param("~match_window", 0.5))
        # Long-axis length of the pallet model. Used to shift gt origin from
        # the geometric centre to the front face centre (where the detector publishes).
        self.pallet_length = rospy.get_param("~pallet_length", 1.22)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)

        self.gt_history = deque(maxlen=300)     # (stamp, x, y, z, yaw)
        self.robot_history = deque(maxlen=300)  # (stamp, x, y, z, yaw)
        self.records = []  # (stamp, range, ex, ey, ez, eyaw_deg)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.gt_cb, queue_size=1)
        rospy.Subscriber("/pallet_detector/pallet_pose", PoseStamped, self.det_cb,
                         queue_size=10)

        rospy.on_shutdown(self.summarize)
        rospy.loginfo("evaluator: comparing /pallet_detector/pallet_pose to gazebo model '%s'",
                      self.gt_model)

    def gt_cb(self, msg: ModelStates):
        now = rospy.Time.now()
        try:
            i = msg.name.index(self.gt_model)
            p = msg.pose[i]
            self.gt_history.append((now,
                                    p.position.x, p.position.y, p.position.z,
                                    yaw_from_quat(p.orientation)))
        except ValueError:
            pass
        try:
            i = msg.name.index(self.robot_model)
            p = msg.pose[i]
            self.robot_history.append((now,
                                       p.position.x, p.position.y, p.position.z,
                                       yaw_from_quat(p.orientation)))
        except ValueError:
            pass

    def _nearest(self, history, stamp):
        if not history:
            return None
        # /gazebo/model_states has no stamp; assume "current". Just take the
        # latest sample within the match window of the detection stamp.
        for s, x, y, z, yaw in reversed(history):
            if abs((s - stamp).to_sec()) <= self.match_window.to_sec():
                return x, y, z, yaw
        return history[-1][1:]  # fall back to most recent

    def nearest_gt(self, stamp):
        return self._nearest(self.gt_history, stamp)

    def nearest_robot(self, stamp):
        return self._nearest(self.robot_history, stamp)

    def det_cb(self, msg: PoseStamped):
        # Previous body — replaced 2026-05-23. Used world_to_base TF, which
        # is a static identity in the duna launch and so silently returned the
        # pose in the robot's base frame as if it were world coords. Worked
        # only when the robot was at world origin with yaw=0.
        # try:
        #     pose_world = self.tf_buf.transform(msg, self.world_frame,
        #                                        timeout=rospy.Duration(0.1))
        # except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
        #         tf2_ros.ConnectivityException) as e:
        #     rospy.logwarn_throttle(2.0, "tf transform failed: %s", e)
        #     return
        # gt = self.nearest_gt(msg.header.stamp)
        # ... (rest of old comparison)

        # 1) Camera->face distance from the DETECTION. The pose is published in
        #    the camera optical frame, so msg.pose.position IS the camera->face
        #    vector in optical coords and its magnitude is the euclidean range.
        det_cam_to_face = (msg.pose.position.x,
                           msg.pose.position.y,
                           msg.pose.position.z)
        det_dist = math.sqrt(sum(c * c for c in det_cam_to_face))

        # 2) Transform detection to the robot's base frame. The URDF chain
        #    (optical -> ... -> base) is correct; only world<-base is broken.
        try:
            pose_base = self.tf_buf.transform(msg, self.robot_frame,
                                              timeout=rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            rospy.logwarn_throttle(2.0, "tf optical->%s failed: %s",
                                   self.robot_frame, e)
            return

        # 3) Get the robot and pallet world poses from gazebo (ground truth).
        robot = self.nearest_robot(msg.header.stamp)
        gt = self.nearest_gt(msg.header.stamp)
        if robot is None or gt is None:
            return
        rx, ry, rz, ryaw = robot
        gt_x, gt_y, gt_z, gt_yaw = gt

        # 4) Compose pose_base -> pose_world using the robot's gazebo pose.
        #    Assumes a ground robot (yaw-only); roll/pitch ignored.
        cyaw = math.cos(ryaw)
        syaw = math.sin(ryaw)
        bx = pose_base.pose.position.x
        by = pose_base.pose.position.y
        bz = pose_base.pose.position.z
        pose_world_x = rx + cyaw * bx - syaw * by
        pose_world_y = ry + syaw * bx + cyaw * by
        pose_world_z = rz + bz
        robot_quat = (0.0, 0.0, math.sin(ryaw * 0.5), math.cos(ryaw * 0.5))
        base_q = pose_base.pose.orientation
        pose_world_quat = tft.quaternion_multiply(
            robot_quat, (base_q.x, base_q.y, base_q.z, base_q.w))

        # 5) Ground-truth pallet front-face pose in world (centre -> front face
        #    along gt -X; orientation re-labeled to detector convention).
        half_L = 0.5 * self.pallet_length
        gt_face_x = gt_x - half_L * math.cos(gt_yaw)
        gt_face_y = gt_y - half_L * math.sin(gt_yaw)
        gt_face_z = gt_z
        gt_quat = (0.0, 0.0, math.sin(gt_yaw * 0.5), math.cos(gt_yaw * 0.5))
        gt_face_quat = tft.quaternion_multiply(gt_quat, _R_CONV_QUAT)
        gt_face_yaw = _yaw_from_quat_xyzw(*gt_face_quat)

        # 6) Camera->face distance from GROUND TRUTH. Camera world position =
        #    robot world pose * base->optical URDF static transform.
        try:
            b2o = self.tf_buf.lookup_transform(
                self.robot_frame, msg.header.frame_id,
                msg.header.stamp, timeout=rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            rospy.logwarn_throttle(2.0, "tf %s->%s failed: %s",
                                   self.robot_frame, msg.header.frame_id, e)
            return
        cam_b = b2o.transform.translation
        cam_world_x = rx + cyaw * cam_b.x - syaw * cam_b.y
        cam_world_y = ry + syaw * cam_b.x + cyaw * cam_b.y
        cam_world_z = rz + cam_b.z
        gt_cam_to_face = (gt_face_x - cam_world_x,
                          gt_face_y - cam_world_y,
                          gt_face_z - cam_world_z)
        gt_dist = math.sqrt(sum(c * c for c in gt_cam_to_face))

        # 7) Pose error in world.
        dx = pose_world_x - gt_face_x
        dy = pose_world_y - gt_face_y
        dz = pose_world_z - gt_face_z
        det_yaw = _yaw_from_quat_xyzw(*pose_world_quat)
        eyaw_deg = math.degrees(wrap_pi(det_yaw - gt_face_yaw))

        # range from camera to pallet centre (world), for the per-frame log.
        rng = math.hypot(gt_x - cam_world_x, gt_y - cam_world_y)
        # Old tuple schema (replaced 2026-05-23):
        # self.records.append((msg.header.stamp.to_sec(), rng, dx, dy, dz, eyaw_deg))
        self.records.append({
            "stamp":       msg.header.stamp.to_sec(),
            "range_m":     rng,
            "dx_m":        dx,
            "dy_m":        dy,
            "dz_m":        dz,
            "yaw_err_deg": eyaw_deg,
            "det_x_w":     pose_world_x,
            "det_y_w":     pose_world_y,
            "det_z_w":     pose_world_z,
            "gt_x_w":      gt_face_x,
            "gt_y_w":      gt_face_y,
            "gt_z_w":      gt_face_z,
            "robot_x_w":   rx,
            "robot_y_w":   ry,
            "robot_yaw_w": ryaw,
        })

        rospy.loginfo(
            "frame@%.2fs  cam->face det=(%+.3f,%+.3f,%+.3f|%.3fm)  "
            "gt=(%+.3f,%+.3f,%+.3f|%.3fm)  "
            "err: dx=%+.3f dy=%+.3f dz=%+.3f yaw=%+.2fdeg",
            msg.header.stamp.to_sec(),
            det_cam_to_face[0], det_cam_to_face[1], det_cam_to_face[2], det_dist,
            gt_cam_to_face[0], gt_cam_to_face[1], gt_cam_to_face[2], gt_dist,
            dx, dy, dz, eyaw_deg)

    def summarize(self):
        n = len(self.records)
        if n == 0:
            rospy.logwarn("evaluator: no matched frames")
            return

        # Old tuple-based slicing (replaced 2026-05-23 when records became dicts):
        # arr = np.array([r[2:] for r in self.records])  # dx, dy, dz, eyaw
        arr = np.array([[r["dx_m"], r["dy_m"], r["dz_m"], r["yaw_err_deg"]]
                        for r in self.records])
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
            # Old positional-tuple writer (replaced 2026-05-23):
            # with open(path, "w", newline="") as f:
            #     w = csv.writer(f)
            #     w.writerow(["stamp", "range_m", "dx_m", "dy_m", "dz_m", "yaw_err_deg"])
            #     w.writerows(self.records)
            fieldnames = ["stamp", "range_m",
                          "dx_m", "dy_m", "dz_m", "yaw_err_deg",
                          "det_x_w", "det_y_w", "det_z_w",
                          "gt_x_w", "gt_y_w", "gt_z_w",
                          "robot_x_w", "robot_y_w", "robot_yaw_w"]
            with open(path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=fieldnames)
                w.writeheader()
                w.writerows(self.records)
            rospy.loginfo("evaluator: wrote %d rows to %s", n, path)


if __name__ == "__main__":
    rospy.init_node("pallet_pose_evaluator")
    PalletPoseEvaluator()
    rospy.spin()
