#!/usr/bin/env python3
"""
Publish the world -> base_link TF from Gazebo link states.

All other frames (base_link -> stand_link -> rgbd_camera_link -> optical frames)
are published by robot_state_publisher from the URDF.

When you later replace the static stand with a mobile robot, just change
what publishes world -> base_link (e.g. odometry / AMCL).
"""
import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import TransformStamped
import tf2_ros

STAND_LINK = "camera_stand::stand_link"


def main():
    rospy.init_node("camera_stand_tf_publisher")

    br = tf2_ros.TransformBroadcaster()

    # Gazebo reports stand_link at z=0.25 (box center).
    # We project to ground level so base_link sits at z=0.
    STAND_LINK_Z_OFFSET = 0.25  # half the 0.5m stand height

    state = {"idx": None, "pose": None, "rot": None}

    def cb(msg: LinkStates):
        if state["idx"] is None:
            try:
                state["idx"] = msg.name.index(STAND_LINK)
            except ValueError:
                rospy.logwarn_throttle(5.0, f"Link '{STAND_LINK}' not found in /gazebo/link_states")
                return

        i = state["idx"]
        state["pose"] = msg.pose[i].position
        state["rot"]  = msg.pose[i].orientation

    def pub_timer(event):
        if state["pose"] is None:
            return
        p = state["pose"]
        o = state["rot"]

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z - STAND_LINK_Z_OFFSET  # project to ground
        t.transform.rotation = o
        br.sendTransform(t)

    rospy.Subscriber("/gazebo/link_states", LinkStates, cb, queue_size=1)
    rospy.Timer(rospy.Duration(1.0/30.0), pub_timer)
    rospy.spin()


if __name__ == "__main__":
    main()
