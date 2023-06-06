#ifndef TARGET_DETECTOR__TARGET_DETECTOR_H
#define TARGET_DETECTOR__TARGET_DETECTOR_H

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <tf2_eigen/tf2_eigen.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

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
		ros::Subscriber reflector_subscriber__;
		ros::Publisher detector_publisher__;

		ros::ServiceServer enable_server__;
		bool enabled__;
		double reflector_baseline__;

		double reflector_distance_tolerance__;

	public:

		bool init();

	protected:

		void reflectorCallback(const reflector_finder::Reflectors & __reflectors);
		bool enableCallback(target_detector::Detector::Request & __request, target_detector::Detector::Response & __response);
		double computeBaseline(const Eigen::Vector3d & __reflector_one, const Eigen::Vector3d & __reflector_two);
};

}

#endif
