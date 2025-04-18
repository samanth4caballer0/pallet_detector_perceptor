#ifndef TARGET_DETECTOR_NODE_LIDAR_2D_H
#define TARGET_DETECTOR_NODE_LIDAR_2D_H

// std
#include <memory>

// ros
#include <sensor_msgs/LaserScan.h>
#include <sick_safetyscanners/ExtendedLaserScanMsg.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
//#include <ar_track_alvar_msgs/AlvarMarkers.h>

// this package
#include "target_detector/node_base.h"
#include "target_detector/detector_reflector.h"
#include "target_detector/detector_column.h"
#include <target_detector/target_detectorConfig.h>

namespace TargetDetector
{

class NodeLidar2d : public NodeBase
{
	protected:

		// ROS API
		std::vector<ros::Subscriber> lidar_subscribers__;
		ros::Subscriber alvar_markers_subscriber__;
		ros::Publisher viz_marker_publisher__;
		dynamic_reconfigure::Server<target_detector::target_detectorConfig> reconfigure_server__;
		dynamic_reconfigure::Server<target_detector::target_detectorConfig>::CallbackType reconfigure_callback__;
		bool first_dynamic_reconfigure__ = true;
		std::mutex reconfigure_mutex__;

		// Detector and detections
		std::shared_ptr<DetectorLidar2d> detector__;
		std::vector<double> detections__;

	public:

		NodeLidar2d();
		~NodeLidar2d();
		bool init();

	protected:

		void laserScanCallback(const sensor_msgs::LaserScanConstPtr & __scan_ptr);
		void extendedLaserScanCallback(const sick_safetyscanners::ExtendedLaserScanMsgConstPtr & __extended_scan_ptr);
		void reconfigureCallback(target_detector::target_detectorConfig & __config, uint32_t __level);
		void publishDetections(const std_msgs::Header & __header);
		void publishMarkers(const std_msgs::Header & __header);
};

}

#endif
