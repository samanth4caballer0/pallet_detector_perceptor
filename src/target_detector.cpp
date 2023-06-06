#include "target_detector/target_detector.h"

namespace TargetDetector
{

bool TargetDetector::init()
{
	if ( !nh__.getParam("reflector_distance_tolerance", reflector_distance_tolerance__) )
	{
		ROS_ERROR("Failed to get reflector_distance_tolerance parameter");
		return false;
	}

	reflector_subscriber__ = nh__.subscribe("reflectors", 1, &TargetDetector::reflectorCallback, this);
	detector_publisher__ = nh__.advertise<target_detector::Detections>( "target_detections", 1, false );
	enable_server__ = nh__.advertiseService("enable_detector", &TargetDetector::enableCallback, this);
	enabled__ = false;

	return true;
}

void TargetDetector::reflectorCallback(const reflector_finder::Reflectors & __reflectors)
{
	if ( !enabled__ )
		return;

	target_detector::Detections detections;
	detections.header = __reflectors.header;

	if ( __reflectors.reflectors.empty() )
	{		
		detector_publisher__.publish(detections);
		return;
	}

	// check all reflector pairs for correct baseline
	for ( int i = 0; i < __reflectors.reflectors.size(); i++)
	{
		for ( int j = i+1; j < __reflectors.reflectors.size(); j++)
		{
			Eigen::Vector3d reflector_one(__reflectors.reflectors.at(i).centroid.x, __reflectors.reflectors.at(i).centroid.y, 1.0);
			Eigen::Vector3d reflector_two(__reflectors.reflectors.at(j).centroid.x, __reflectors.reflectors.at(j).centroid.y, 1.0);

			double actual_baseline = computeBaseline(reflector_one, reflector_two);
			if ( std::abs( actual_baseline - reflector_baseline__ ) < reflector_distance_tolerance__ )
			{
				// we have a detection, so compute marker frame
				Eigen::Vector3d x_axis;
				Eigen::Vector3d y_axis;
				Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
				Eigen::Vector3d platform_to_marker;

				y_axis = reflector_two - reflector_one;
				x_axis = y_axis.cross(z_axis);
				platform_to_marker = reflector_one;
				if ( x_axis.dot(platform_to_marker) > 0.0 ) // we need to switch axes
				{
					y_axis = -y_axis;
					x_axis = y_axis.cross(z_axis);
					platform_to_marker = reflector_two;
				}

				target_detector::Detection detection;
				detection.pose.position = tf2::toMsg(platform_to_marker);
				double angle = std::atan2(x_axis.y(), x_axis.x());
				detection.pose.orientation.z = std::sin(angle/2.0);
				detection.pose.orientation.w = std::cos(angle/2.0);
				detection.supports.push_back(__reflectors.reflectors.at(i).centroid);
				detection.supports.push_back(__reflectors.reflectors.at(j).centroid);
				detection.baseline = actual_baseline;

				detections.detections.push_back(detection);		
			}
		}
	}
	detector_publisher__.publish(detections);
}

double TargetDetector::computeBaseline(const Eigen::Vector3d & __reflector_one, const Eigen::Vector3d & __reflector_two)
{
	return	(__reflector_one-__reflector_two).norm();
}

bool TargetDetector::enableCallback(target_detector::Detector::Request & __request, target_detector::Detector::Response & __response)
{
	if ( __request.reflector_baseline < 0.0)
	{
		__response.success = false;
		return true;
	}

	enabled__ = __request.enable;
	reflector_baseline__ = __request.reflector_baseline;
	__response.success = true;
	return true;
}

}
