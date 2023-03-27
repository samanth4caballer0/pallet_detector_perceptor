#ifndef TARGET_DETECTOR_TARGET_DETECTOR_NODE_H
#define TARGET_DETECTOR_TARGET_DETECTOR_NODE_H

// std
#include <memory>

//ros
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <sick_safetyscanners/ExtendedLaserScanMsg.h>

//Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//this project
#include "marker_reflector/cluster.h"
#include "marker_reflector/marker_reflector.h"
#include "target_detector/DetectAction.h"
#include "target_detector/Detections.h"


const int DETECTOR_MARKER_REFLECTOR = 1;
const int DETECTOR_STD_PALLET = 2; // not implemented
const int DETECTOR_QR = 3; // not implemented

const int SENSOR_MSG_LASER_SCAN = 1;
const int SICK_EXTENDED_LASER_SCAN = 2;

namespace TargetDetector
{

class TargetDetectorNode
{
	protected:
		// detector object
		std::shared_ptr<DetectorBase> detector__;

		// type of detector object
		int detector_type__;

		// type of laser scan topic (1:sensor_msgs/LaserScan ;  2:sick_safetyscanners/ExtendedLaserScanMsg)
		int topic_type__;

		//ros node handle
		ros::NodeHandle nh__;

		// detector action
		std::shared_ptr<actionlib::SimpleActionServer<target_detector::DetectAction>> detect_as_ptr__;

		//subscribers
		ros::Subscriber lidar_reflector_subscriber__;

		//publishers
		ros::Publisher detector_publisher__;
		ros::Publisher viz_marker_publisher__;

		//Transfrom listener to get transform from platform to sensor_frame__
		tf::TransformListener tfl__;

		// 2D sensor frame wrt the platform frame
		Eigen::Isometry2d T_platform2sensor__;

		//detector and node parameters
		std::map<std::string, std::string> lidar_frame_to_topic_map__;
		std::string sensor_frame__; // frame of the sensor providing data
		double reflector_intensity_threshold__;
		std::map<std::string, double> dynamic_params__; //parameters that can be changed for each new detection
		bool verbose__;

		// action feedback
		std::atomic<unsigned char> detector_state__;
		std::atomic<bool> detecting_flag__;

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
		void laserScanCallback(const sensor_msgs::LaserScan & __scan);
		void laserScanExtendedCallback(const sick_safetyscanners::ExtendedLaserScanMsg & __scan);
		void detect(const ros::Time & __stamp);

		// publish visualization markers for debugging purposes
		void publishMarkers(
			const std::vector<Eigen::Vector3d> & __key_points,
			const std::vector<Eigen::Vector3d> & __positions,
			const std::vector<Eigen::Quaterniond> & __orientations,
			const std::string __marker_namespace) const;
};

} //namespace

#endif
