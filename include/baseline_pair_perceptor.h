#ifndef TARGET_DETECTOR__BASELINE_PAIR_PERCEPTOR_H
#define TARGET_DETECTOR__BASELINE_PAIR_PERCEPTOR_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>

#include <target_detector/DetectorEnable.h>
#include <target_detector/Detections.h>

namespace TargetDetector
{

class BaselinePairPerceptor
{
	protected:

		ros::NodeHandle nh__;

		ros::Publisher detections_out_publisher__;
		ros::ServiceServer enable_server__;

		ros::Subscriber detections_in_subscriber__;

		bool enabled__ = false;
		double baseline_tolerance__ = 0.0;
		double baseline__ = 0.0;

	public:

		bool init();

	protected:

		bool enableCallback(target_detector::DetectorEnable::Request & __request, target_detector::DetectorEnable::Response & __response);
		bool configureParameters();

		void detectionsInCallback(const target_detector::Detections & __detections_in);
		void subscribeToData();
		void unsubscribeFromData();

		template <typename T>
		bool getParamOrFail(const std::string & __name, T& __variable)
		{
			if ( !nh__.getParam(__name, __variable) )
			{
				ROS_ERROR_STREAM("Failed to get parameter: " << __name);
				return false;
			}
			return true;
		};

};
	
}

#endif