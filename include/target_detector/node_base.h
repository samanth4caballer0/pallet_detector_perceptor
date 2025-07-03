#ifndef TARGET_DETECTOR_NODE_BASE_H
#define TARGET_DETECTOR_NODE_BASE_H

// std
#include <iostream>

// eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// ros
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// this package
#include "target_detector/Detections.h"

namespace TargetDetector
{

class NodeBase
{
	protected:

		// ROS API
		ros::NodeHandle nh__;
		ros::Publisher detector_publisher__;
		ros::Publisher viz_marker_publisher__;

		// config node parameters
		bool enable__;
		uint8_t detector_type__;
		std::string robot_frame__;
		bool vizbose__; // if true, publishes visualization markers
		std::map<std::string, double> detector_params__;

		// tf
		tf2_ros::Buffer tf_buffer__;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener__;
		std::map<std::string, Eigen::Isometry3d> T_robot_to_sensor__;
		std::map<std::string, Eigen::Isometry2d> T_robot_to_sensor_2d__;

	public:

		NodeBase();
		~NodeBase();
		bool init();

	protected:

		bool saveSensorTransform(const std_msgs::Header & __header);
};

}

#endif
