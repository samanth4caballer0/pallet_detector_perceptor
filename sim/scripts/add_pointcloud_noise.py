#!/usr/bin/env python3
"""
Add range-direction Gaussian noise to a depth point cloud.

Models D435-style axial noise: stddev(z) = base + coeff * z^2.
Each point is perturbed along its ray (origin = sensor optical frame),
which matches how depth-camera noise actually behaves.

Why this script instead of `pal-robotics/realsense_gazebo_plugin`?
We tried the plugin (melodic-devel branch, compiled clean on Noetic +
Gazebo 11). Its source (`gazebo_ros_realsense.cpp::FillPointCloudHelper`)
copies Gazebo's raw depth straight into the cloud — same logic as
`libgazebo_ros_depth_camera.so`, just repackaged with 4 cameras
(color + ired1 + ired2 + depth). No noise model, no edge artifacts,
no IR-stereo disparity; IR1/IR2 render but are never used for depth.
For noise realism this script is strictly better (depth-dependent axial
stddev). The plugin's only real value is topic-name parity with the
real D435 driver, which matters only when porting to real hardware.
Don't re-run that experiment expecting different results.
"""
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2


class PointCloudNoiser:
    def __init__(self):
        self.base_stddev = rospy.get_param("~base_stddev", 0.005)
        self.range_coeff = rospy.get_param("~range_coeff", 0.002)
        self.input_topic = rospy.get_param("~input", "/demo/rgbd_camera/depth/points")
        self.output_topic = rospy.get_param("~output", "/demo/rgbd_camera/depth/points_noisy")

        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.cb, queue_size=1)
        self.rng = np.random.default_rng()

        rospy.loginfo("noise: %s -> %s  base=%.4fm  coeff=%.4f/m",
                      self.input_topic, self.output_topic,
                      self.base_stddev, self.range_coeff)

    def cb(self, msg: PointCloud2):
        pts = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"),
                                            skip_nans=True)), dtype=np.float32)
        if pts.size == 0:
            return

        r = np.linalg.norm(pts, axis=1)
        valid = r > 1e-6
        sigma = self.base_stddev + self.range_coeff * r * r
        delta = self.rng.normal(0.0, 1.0, size=r.shape).astype(np.float32) * sigma
        scale = np.where(valid, (r + delta) / np.where(valid, r, 1.0), 1.0)
        noisy = pts * scale[:, None]

        header = msg.header
        fields = [
            PointField("x", 0,  PointField.FLOAT32, 1),
            PointField("y", 4,  PointField.FLOAT32, 1),
            PointField("z", 8,  PointField.FLOAT32, 1),
        ]
        out = pc2.create_cloud(header, fields, noisy.tolist())
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("pointcloud_noiser")
    PointCloudNoiser()
    rospy.spin()
