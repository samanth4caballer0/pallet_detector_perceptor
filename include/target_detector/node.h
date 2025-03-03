#ifndef TARGET_DETECTOR__NODE_H
#define TARGET_DETECTOR__NODE_H

// std
#include <memory>

// ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <sick_safetyscanners/ExtendedLaserScanMsg.h>

// this package
#include "target_detector/detector_reflector.h"
#include "target_detector/detector_column.h"
#include "target_detector/Detections.h"
#include <target_detector/target_detectorConfig.h>

namespace TargetDetector
{

class Node
{
	protected:

		// ROS API
		ros::NodeHandle nh__;
		std::vector<ros::Subscriber> lidar_subscribers__;
		ros::Publisher detetctor_publisher__;
		ros::Publisher viz_marker_publisher__;
		dynamic_reconfigure::Server<target_detector::target_detectorConfig> reconfigure_server__;
		dynamic_reconfigure::Server<target_detector::target_detectorConfig>::CallbackType reconfigure_callback__;
		bool first_dynamic_reconfigure__ = true;
		std::mutex reconfigure_mutex__;

		// config node parameters
		std::string robot_frame__;
		std::map<std::string, double> detector_params__;

		// tf
		tf2_ros::Buffer tf_buffer__;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener__;
		//std::map<std::string, geometry_msgs::TransformStamped> T_lidar_to_robot__;
		std::map<std::string, Eigen::Isometry2d> T_robot_to_lidar__;

		// Detector and detections
		//TargetDetector::DetectorReflector detector__;
		std::shared_ptr<DetectorLidar2d> detector__;
		std::vector<double> detections__;

		// params
		bool vizbose__; // if true, publishes visualization markers

	public:

		Node();
		~Node();
		bool init();

	protected:

		void laserScanCallback(const sensor_msgs::LaserScanConstPtr & __scan_ptr);
		void extendedLaserScanCallback(const sick_safetyscanners::ExtendedLaserScanMsgConstPtr & __extended_scan_ptr);
		void reconfigureCallback(target_detector::target_detectorConfig & __config, uint32_t __level);
		bool saveLidarTransform(const std_msgs::Header & __header);
		void publishDetections(const std_msgs::Header & __header);
		void publishMarkers(const std_msgs::Header & __header);
};

}

#endif
