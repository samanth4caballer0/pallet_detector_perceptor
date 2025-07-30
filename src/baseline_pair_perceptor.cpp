#include "baseline_pair_perceptor.h"

namespace TargetDetector
{

bool BaselinePairPerceptor::init()
{

	if ( !configureParameters() )
	{
		ROS_ERROR("Failed to configure baseline pair detection");
		return false;
	}

	detections_out_publisher__ = nh__.advertise<target_detector::Detections>("detections", 1, false);
	enable_server__ = nh__.advertiseService("enable", &BaselinePairPerceptor::enableCallback, this);

	return true;
};

void BaselinePairPerceptor::detectionsInCallback(const target_detector::Detections & __detections_in)
{
	if ( !enabled__ )
		return;

	// we keep frame and timestamp of the detections from sensors
	target_detector::Detections detections_out;
	detections_out.header = __detections_in.header;

	// check all detection pairs for correct baseline
	double actual_baseline, angle;
	Eigen::Vector3d detection_one;
	Eigen::Vector3d detection_two;
	Eigen::Vector3d x_axis;
	Eigen::Vector3d y_axis;
	Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
	Eigen::Vector3d platform_to_marker;
	target_detector::Detection detection;
	detection.type = target_detector::Detection::TYPE_REFLECTOR_FROM_INTENSITY;
	detection.id = -1;
	detection.pose.pose.position.z = 0.0;
	detection.pose.pose.orientation.x = 0.0;
	detection.pose.pose.orientation.y = 0.0;
	detection.supports = 2;

	for ( int ii = 0; ii < __detections_in.detections.size(); ii++)
	{
		for ( int jj = ii+1; jj < __detections_in.detections.size(); jj++)
		{
			detection_one << __detections_in.detections[ii].pose.pose.position.x, __detections_in.detections[ii].pose.pose.position.y, 1.0;
			detection_two << __detections_in.detections[jj].pose.pose.position.x, __detections_in.detections[jj].pose.pose.position.y, 1.0;
			actual_baseline = (detection_one - detection_two).norm();
			if ( std::abs( actual_baseline - baseline__ ) < baseline_tolerance__ )
			{
				// we have a good detection pair, so compute marker frame
				y_axis = detection_two - detection_one;
				x_axis = y_axis.cross(z_axis);
				platform_to_marker = detection_one;
				if ( x_axis.dot(platform_to_marker) > 0.0 ) // we need to switch axes
				{
					y_axis = -y_axis;
					x_axis = y_axis.cross(z_axis);
					platform_to_marker = detection_two;
				}
				angle = std::atan2(x_axis.y(), x_axis.x());

				detection.pose.pose.position.x = platform_to_marker.x();
				detection.pose.pose.position.y = platform_to_marker.y();
				detection.pose.pose.orientation.z = std::sin(angle/2.0);
				detection.pose.pose.orientation.w = std::cos(angle/2.0);
				detection.points.clear();
				detection.points.push_back(__detections_in.detections[ii].pose.pose.position);
				detection.points.push_back(__detections_in.detections[jj].pose.pose.position);
				detection.baseline = actual_baseline;

				detections_out.detections.push_back(detection);
			}
		}
	}

	detections_out_publisher__.publish(detections_out);
}

bool BaselinePairPerceptor::enableCallback(target_detector::DetectorEnable::Request & __request, target_detector::DetectorEnable::Response & __response)
{
	if ( __request.enable && __request.baseline <= 0.0 )
	{
		ROS_ERROR_STREAM("Invalid baseline for pair detection: " << __request.baseline);
		__response.success = false;
		return true;
	}

	// we update baseline, even if already enabled
	if ( __request.enable )
		baseline__ = __request.baseline;

	if ( !enabled__ && __request.enable )
		subscribeToData();

	if ( enabled__ && !__request.enable )
		unsubscribeFromData();

	enabled__ = __request.enable;
	__response.success = true;
	return true;
}

bool BaselinePairPerceptor::configureParameters()
{
	return	getParamOrFail("baseline_tolerance", baseline_tolerance__);
}

void BaselinePairPerceptor::subscribeToData()
{
	detections_in_subscriber__ = nh__.subscribe("detections_in", 1, &BaselinePairPerceptor::detectionsInCallback, this);
}

void BaselinePairPerceptor::unsubscribeFromData()
{
	detections_in_subscriber__.shutdown();
}

}