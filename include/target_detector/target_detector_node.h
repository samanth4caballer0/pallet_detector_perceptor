#ifndef TARGET_DETECTOR_TARGET_DETECTOR_NODE_H
#define TARGET_DETECTOR_TARGET_DETECTOR_NODE_H

// std
#include <memory>

//ros
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
//#include <sick_safetyscanners/ExtendedLaserScanMsg.h>

//this project
#include "marker_reflector/cluster.h"
#include "marker_reflector/marker_reflector.h"
#include "target_detector/DetectAction.h"


const int DETECTOR_MARKER_REFLECTOR = 1;
const int DETECTOR_STD_PALLET = 2; // not implemented
const int DETECTOR_QR = 3; // not implemented

namespace TargetDetector
{

class TargetDetectorNode
{
	protected:
		// detector object
		std::shared_ptr<DetectorBase> detector__;

		//ros node handle
		ros::NodeHandle nh__;

		//enabling/disabling tracking through service
		std::shared_ptr<actionlib::SimpleActionServer<target_detector::DetectAction>> detect_as_ptr__;

		//subscribers
		ros::Subscriber lidar_subscriber__;

		//publishers
		ros::Publisher detector_publisher__;
		ros::Publisher viz_publisher__;

		//Transfrom listener to get marker frame and update ROI
		tf::TransformListener tfl__;
		tf::TransformBroadcaster tfb__;

		//detector parameters
		std::map<std::string, std::string> lidar_frame_to_topic_map__;
		std::string sensor_frame__; //lidar frame in which detection is referenced
		std::string marker_frame__; // updated marker frame (required for ROI update)
		bool verbose__;

	public:
		//constructor
		TargetDetectorNode();

		//destructor
		~TargetDetectorNode();

		//inits params
		bool init();

	protected:
		// detect action callback
		void detectCallback(const target_detector::DetectGoalConstPtr & __goal);

		// Laser scan callback. Detection starts here
		//void lidarCallback(const sick_safetyscanners::ExtendedLaserScanMsg & __scan);
		void lidarCallback(const sensor_msgs::LaserScan & __scan);

		// publish visualization markers for debugging purposes
		// __red_color must be in [0,1]
		void publishMarkers(
			const std::vector<Cluster> & __clusters,
			const double & __red_color,
			const std::string __marker_namespace) const;
};

} //namespace

#endif
