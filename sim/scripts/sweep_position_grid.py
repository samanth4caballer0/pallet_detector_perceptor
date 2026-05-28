#!/usr/bin/env python3
"""
Position-grid envelope sweep for the pallet detector.

Teleports the robot through a 7x7 grid of spawn positions in front of the
pallet (distance d in [1.0, 4.0] m, lateral y in [-1.5, +1.5] m, both in
0.5 m steps). At each cell the robot's world yaw is set so the camera
points at the pallet centre (relative yaw = 0 across the entire sweep).
After a short settle, the node averages the detector's pallet pose vs
Gazebo ground truth over a few detection samples and records mean xy and
yaw errors. Writes one CSV row per cell, suitable for the heatmaps in
plot_envelope_heatmap.py.

The detector frame composition mirrors evaluate_pallet_pose.py: optical
pose -> robot base via URDF TF, then composed to world using the robot's
gazebo pose (bypassing the launch's identity world<-base static TF).

Usage:
  rosrun target_detector sweep_position_grid.py _csv:=/tmp/sweep.csv
  # or via launch:
  roslaunch target_detector sweep_envelope.launch sweep_csv:=/tmp/sweep.csv
"""
import csv
import math
import os
import sys

import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transform)
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest

# Shared math helpers — single source of truth for the convention rotation
# and the wrap-pi / yaw-from-quaternion utilities.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)
from evaluate_pallet_pose import (  # noqa: E402
    _R_CONV_QUAT, _yaw_from_quat_xyzw, yaw_from_quat, wrap_pi,
)

# Default pallet long-axis -> stringer-face calibration. Same value used by
# evaluate_pallet_pose.py; see the comment there for the derivation.
DEFAULT_PALLET_LENGTH = 1.172


class PositionGridSweep:
    def __init__(self):
        # ---- Trapezoidal grid (4 rows × 4 cells = 16 cells) ----
        # Designed from corner-test logs (2026-05-28). Each row has its own
        # lateral span so cells stay within (or just at) the camera's
        # detection envelope — the rectangular 7×7 grid earlier wasted most
        # of its cells outside the FoV. Coordinates are
        # (distance to pallet centre [m], [robot world-Y values [m]]).
        # The 1.172 m face calibration puts the pallet face at world
        # x ≈ 1.414 m, so cam-to-face X distances per row are
        # 0.87, 1.40, 1.90, 2.40 m respectively.
        # Previous (rectangular) grid:
        # self.distances = [1.4, 1.8, 2.2, 2.6, 3.0, 3.4, 3.8]
        # self.laterals  = [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5]
        self.grid_rows = [
            (1.54, [-0.4,   -0.13,  +0.13,  +0.4]),                # cam→face X ≈ 0.95 m  (4 cells, ±0.4 lateral)
            (1.99, [-0.8,   -0.4,    0.0,   +0.4,   +0.8]),       # cam→face X ≈ 1.40 m  (5 cells, tightened ±0.8)
            (2.49, [-0.9,   -0.3,   +0.3,   +0.9]),               # cam→face X ≈ 1.90 m  (4 cells)
            (2.99, [-1.05,  -0.35,  +0.35,  +1.05]),              # cam→face X ≈ 2.40 m  (4 cells)
        ]

        # ---- Pallet world position (read from the world file) ----
        self.pallet_x = rospy.get_param("~pallet_x", 2.0)
        self.pallet_y = rospy.get_param("~pallet_y", 0.0)

        # ---- Model & frame names ----
        self.gt_model      = rospy.get_param("~gt_model",     "pallet")
        self.robot_model   = rospy.get_param("~robot_model",  "duna")
        self.robot_frame   = rospy.get_param("~robot_frame",  "base")
        self.pallet_length = rospy.get_param("~pallet_length", DEFAULT_PALLET_LENGTH)

        # ---- Robot world yaw (constant across the entire sweep) ----
        # In the first (position-only) sweep this is 0 deg so every spawn is
        # parallel to the pallet face; only (x, y) changes between cells.
        # A second sweep can hold the same grid at a non-zero constant yaw
        # (e.g. robot_yaw_deg:=10) to characterise the off-axis envelope.
        self.robot_yaw_rad = (
            math.radians(float(rospy.get_param("~robot_yaw_deg", 0.0))))

        # ---- Timing ----
        # Two-phase per-cell: a fixed settle, then wait for the first
        # detection (with a timeout), then collect a small additional
        # window so we can average a few samples instead of relying on
        # one. Successful cells finish in ~settle + few * 1/rate; cells
        # outside the envelope time out at `detection_timeout`.
        self.settle_time  = float(rospy.get_param("~settle_time",  3.0))
        # Max time to wait for the first detection after settle. If no
        # pallet_pose arrives within this, the cell is marked failed and
        # the sweep moves on. Replaces the old fixed `collect_time`.
        # 12 s chosen because far-row cells often publish only every
        # 5-8 s (the perceptor rejects most frames at low signal). 7 s
        # was timing out just before legitimate late detections.
        self.detection_timeout = float(
            rospy.get_param("~detection_timeout", 12.0))
        # After the first detection arrives, keep collecting for this
        # long to gather extra samples for per-cell averaging.
        self.post_first_collect = float(
            rospy.get_param("~post_first_collect", 1.5))
        # min_samples=1: at the actual ~1 Hz publish rate, even a single
        # detection at a cell is a useful data point for the envelope
        # heatmap. Cells with zero detections will still show "no det".
        self.min_samples  = int(rospy.get_param("~min_samples", 1))
        self.max_samples  = int(rospy.get_param("~max_samples", 10))

        # ---- Output ----
        self.csv_path = rospy.get_param("~csv", "/tmp/sweep.csv")

        # ---- TF + service ----
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)
        rospy.loginfo("sweep: waiting for /gazebo/set_model_state ...")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        # ---- State ----
        self.last_pallet_gt = None     # (x, y, z, yaw)
        self.last_robot_gt  = None
        self.det_buffer     = []       # list of (PoseStamped, robot_gt, pallet_gt)
        self.collecting     = False

        # ---- Subscribers ----
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self.gt_cb, queue_size=1)
        rospy.Subscriber("/pallet_detector/pallet_pose", PoseStamped,
                         self.det_cb, queue_size=10)

        rospy.loginfo(
            "sweep: pallet=(%.2f,%.2f) robot_model='%s' grid:",
            self.pallet_x, self.pallet_y, self.robot_model)
        for d, lats in self.grid_rows:
            rospy.loginfo("        d=%.2f m  y=%s", d, lats)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def gt_cb(self, msg: ModelStates):
        try:
            i = msg.name.index(self.gt_model)
            p = msg.pose[i]
            self.last_pallet_gt = (p.position.x, p.position.y, p.position.z,
                                   yaw_from_quat(p.orientation))
        except ValueError:
            pass
        try:
            i = msg.name.index(self.robot_model)
            p = msg.pose[i]
            self.last_robot_gt = (p.position.x, p.position.y, p.position.z,
                                  yaw_from_quat(p.orientation))
        except ValueError:
            pass

    def det_cb(self, msg: PoseStamped):
        if not self.collecting:
            return
        if self.last_robot_gt is None or self.last_pallet_gt is None:
            return
        self.det_buffer.append((msg, self.last_robot_gt, self.last_pallet_gt))

    # ------------------------------------------------------------------
    # Teleport + per-sample comparison (mirrors evaluate_pallet_pose)
    # ------------------------------------------------------------------
    def teleport(self, d, y):
        robot_x = self.pallet_x - d
        robot_y = self.pallet_y + y
        # Constant world yaw across the whole sweep — the robot stays
        # parallel to the pallet face, only translated. Configurable via
        # ~robot_yaw_deg (default 0). Auto-aim-at-pallet removed (replaced
        # 2026-05-28): originally yaw = atan2(pallet_y-robot_y, pallet_x-robot_x).
        yaw = self.robot_yaw_rad

        ms = ModelState()
        ms.model_name      = self.robot_model
        ms.pose.position.x = robot_x
        ms.pose.position.y = robot_y
        ms.pose.position.z = 0.0
        ms.pose.orientation.z = math.sin(yaw * 0.5)
        ms.pose.orientation.w = math.cos(yaw * 0.5)
        ms.reference_frame = "world"
        try:
            resp = self.set_state(SetModelStateRequest(model_state=ms))
            if not resp.success:
                rospy.logwarn("sweep: teleport failed: %s", resp.status_message)
                return False
        except rospy.ServiceException as e:
            rospy.logwarn("sweep: set_model_state ServiceException: %s", e)
            return False
        return True

    def compute_sample_error(self, sample):
        """Return (dx, dy, yaw_err_deg) in world for a single detection."""
        msg, robot_gt, pallet_gt = sample
        rx, ry, rz, ryaw = robot_gt
        gt_x, gt_y, gt_z, gt_yaw = pallet_gt

        # 1) Optical -> robot base via URDF TF (reliable; world<-base is not)
        try:
            pose_base = self.tf_buf.transform(msg, self.robot_frame,
                                              timeout=rospy.Duration(0.2))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None

        # 2) Compose to world using the robot's gazebo pose (yaw-only).
        cyaw, syaw = math.cos(ryaw), math.sin(ryaw)
        bx = pose_base.pose.position.x
        by = pose_base.pose.position.y
        pose_world_x = rx + cyaw * bx - syaw * by
        pose_world_y = ry + syaw * bx + cyaw * by
        robot_quat = (0.0, 0.0, math.sin(ryaw * 0.5), math.cos(ryaw * 0.5))
        bq = pose_base.pose.orientation
        pose_world_quat = tft.quaternion_multiply(
            robot_quat, (bq.x, bq.y, bq.z, bq.w))

        # 3) GT face pose in world (centre -> face along gt -X; convention quat)
        half_L = 0.5 * self.pallet_length
        gt_face_x = gt_x - half_L * math.cos(gt_yaw)
        gt_face_y = gt_y - half_L * math.sin(gt_yaw)
        gt_quat = (0.0, 0.0, math.sin(gt_yaw * 0.5), math.cos(gt_yaw * 0.5))
        gt_face_quat = tft.quaternion_multiply(gt_quat, _R_CONV_QUAT)
        gt_face_yaw = _yaw_from_quat_xyzw(*gt_face_quat)

        dx = pose_world_x - gt_face_x
        dy = pose_world_y - gt_face_y
        det_yaw = _yaw_from_quat_xyzw(*pose_world_quat)
        eyaw_deg = math.degrees(wrap_pi(det_yaw - gt_face_yaw))
        return dx, dy, eyaw_deg

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self):
        # Wait for first GT msg so we know gazebo is up
        rospy.loginfo("sweep: waiting for first /gazebo/model_states ...")
        while not rospy.is_shutdown() and (
                self.last_pallet_gt is None or self.last_robot_gt is None):
            rospy.sleep(0.1)
        if rospy.is_shutdown():
            return
        rospy.loginfo("sweep: GT received, starting %d-cell sweep",
                      sum(len(lats) for _, lats in self.grid_rows))

        # Open CSV up-front and write incrementally so partial sweeps are usable
        path = os.path.expanduser(self.csv_path)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        fieldnames = ["distance_m", "lateral_m", "n",
                      "mean_dx_m", "mean_dy_m", "xy_err_m",
                      "yaw_err_deg", "abs_yaw_err_deg"]
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            f.flush()

            total = sum(len(lats) for _, lats in self.grid_rows)
            cell_idx = 0
            for d, lats in self.grid_rows:
                for y in lats:
                    cell_idx += 1
                    if rospy.is_shutdown():
                        return

                    rospy.loginfo("sweep [%d/%d] teleport d=%.2f y=%+.2f",
                                  cell_idx, total, d, y)

                    if not self.teleport(d, y):
                        writer.writerow(_nan_row(d, y, fieldnames))
                        f.flush()
                        continue

                    rospy.sleep(self.settle_time)

                    # Phase 1: wait for the first detection at this cell,
                    # with a hard timeout. Phase 2: once any detection
                    # arrives, keep collecting for post_first_collect more
                    # seconds to gather a few extra samples for averaging.
                    self.det_buffer = []
                    self.collecting = True
                    deadline = rospy.Time.now() + rospy.Duration(self.detection_timeout)
                    poll = rospy.Rate(20)
                    t_wait_start = rospy.Time.now()
                    while (rospy.Time.now() < deadline
                           and not rospy.is_shutdown()
                           and len(self.det_buffer) == 0):
                        poll.sleep()
                    t_to_first = (rospy.Time.now() - t_wait_start).to_sec()
                    if len(self.det_buffer) > 0:
                        rospy.sleep(self.post_first_collect)
                    self.collecting = False

                    buffered = len(self.det_buffer)
                    samples = self.det_buffer[:self.max_samples]
                    errors = [self.compute_sample_error(s) for s in samples]
                    n_tf_ok = sum(1 for e in errors if e is not None)
                    errors = [e for e in errors if e is not None]
                    n = len(errors)

                    rospy.loginfo("sweep [%d/%d] t_first=%.2fs  buffered=%d  "
                                  "tf_ok=%d  valid=%d",
                                  cell_idx, total, t_to_first,
                                  buffered, n_tf_ok, n)

                    if n < self.min_samples:
                        rospy.logwarn("sweep [%d/%d] only %d samples (need >=%d) — "
                                      "t_first=%.2fs buffered=%d tf_ok=%d",
                                      cell_idx, total, n, self.min_samples,
                                      t_to_first, buffered, n_tf_ok)
                        row = _nan_row(d, y, fieldnames)
                        row["n"] = n
                        writer.writerow(row)
                        f.flush()
                        continue

                    dxs   = np.array([e[0] for e in errors])
                    dys   = np.array([e[1] for e in errors])
                    eyaws = np.array([e[2] for e in errors])
                    mean_xy_err = float(np.mean(np.hypot(dxs, dys)))
                    abs_yaw_err = float(np.mean(np.abs(eyaws)))

                    writer.writerow({
                        "distance_m":      d,
                        "lateral_m":       y,
                        "n":               n,
                        "mean_dx_m":       float(np.mean(dxs)),
                        "mean_dy_m":       float(np.mean(dys)),
                        "xy_err_m":        mean_xy_err,
                        "yaw_err_deg":     float(np.mean(eyaws)),
                        "abs_yaw_err_deg": abs_yaw_err,
                    })
                    f.flush()

                    rospy.loginfo("  -> n=%d  xy_err=%.3f m  |yaw|=%.2f deg",
                                  n, mean_xy_err, abs_yaw_err)

        rospy.loginfo("sweep: complete. CSV at %s", path)


def _nan_row(d, y, fieldnames):
    row = {k: float("nan") for k in fieldnames}
    row["distance_m"] = d
    row["lateral_m"]  = y
    row["n"]          = 0
    return row


if __name__ == "__main__":
    rospy.init_node("position_grid_sweep")
    PositionGridSweep().run()
