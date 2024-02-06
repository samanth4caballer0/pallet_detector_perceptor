#ifndef TARGET_DETECTOR__TARGET_DETECTOR_H
#define TARGET_DETECTOR__TARGET_DETECTOR_H

// eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// ros
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "ar_track_alvar_msgs/AlvarMarkers.h"

// duna
#include "reflector_finder/Reflectors.h"
#include "target_detector/Detections.h"
#include "target_detector/Detection.h"
#include "target_detector/Detector.h"


namespace TargetDetector
{

class TargetDetector
{
	protected:

		ros::NodeHandle nh__;
		ros::ServiceServer mode_server__;
		ros::Subscriber reflector_subscriber__;
		ros::Subscriber alvar_subscriber__;
		ros::Publisher detector_publisher__;

		unsigned int mode__;
		double reflector_baseline__;
		double reflector_distance_tolerance__;
		unsigned int alvar_marker_id__;

		tf2_ros::Buffer tf_buffer__;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener__;

	public:

		bool init();

	protected:

		bool modeCallback(target_detector::Detector::Request & __request, target_detector::Detector::Response & __response);
		void reflectorCallback(const reflector_finder::Reflectors & __reflectors);
		void alvarCallback(const ar_track_alvar_msgs::AlvarMarkers & __alvar_markers);
		double computeBaseline(const Eigen::Vector3d & __reflector_one, const Eigen::Vector3d & __reflector_two);
};

}

#endif
