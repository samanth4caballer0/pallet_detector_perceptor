#include "target_detector/node_baseline_pair.h"

namespace TargetDetector
{

NodeBaselinePair::NodeBaselinePair()
{
	//
}

NodeBaselinePair::~NodeBaselinePair()
{
		//
}

bool NodeBaselinePair::init()
{
	// base init with common configs
	if ( !NodeBase::init() )
	{
		return false;
	}

	// config params
	if ( !nh__.getParam("reflector_baseline_tolerance", detector_params__["reflector_baseline_tolerance"]) )
	{
		ROS_ERROR("Failed to get reflector_baseline_tolerance parameter.");
		return false;
	}

	// specialized subscriber for this class
	detections_subscriber__ = nh__.subscribe("detections_in", 1, &NodeBaselinePair::detectionsCallback, this);

	// visuals
	if ( vizbose__ )
	{
		viz_marker_publisher__ = nh__.advertise<visualization_msgs::MarkerArray>( "visuals", 1, true );
	}

	return true;
}

void NodeBaselinePair::detectionsCallback(const target_detector::Detections & __msg)
{
	if ( !enable__ ) return;

	// output message with the same header as the input one
	target_detector::Detections msg;
	msg.header = __msg.header;

	// check if any alvar detection
	if ( __msg.detections.empty() )
	{
		detector_publisher__.publish(msg);
		return;
	}

	// check all reflector pairs for correct baseline
	double actual_baseline, angle;
	Eigen::Vector3d reflector_one;
	Eigen::Vector3d reflector_two;
	Eigen::Vector3d x_axis;
	Eigen::Vector3d y_axis;
	Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
	Eigen::Vector3d platform_to_marker;
	target_detector::Detection detection;
	double reflector_baseline = detector_params__["reflector_baseline"];
	double reflector_baseline_tolerance = detector_params__["reflector_baseline_tolerance"];
	for ( int ii = 0; ii < __msg.detections.size(); ii++)
	{
		for ( int jj = ii+1; jj < __msg.detections.size(); jj++)
		{
			reflector_one << __msg.detections[ii].pose.pose.position.x, __msg.detections[ii].pose.pose.position.y, 1.0;
			reflector_two << __msg.detections[jj].pose.pose.position.x, __msg.detections[jj].pose.pose.position.y, 1.0;
			actual_baseline = (reflector_one - reflector_two).norm();
			if ( std::abs( actual_baseline - reflector_baseline ) < reflector_baseline_tolerance )
			{
				// we have a good detection pair, so compute marker frame
				y_axis = reflector_two - reflector_one;
				x_axis = y_axis.cross(z_axis);
				platform_to_marker = reflector_one;
				if ( x_axis.dot(platform_to_marker) > 0.0 ) // we need to switch axes
				{
					y_axis = -y_axis;
					x_axis = y_axis.cross(z_axis);
					platform_to_marker = reflector_two;
				}

				// fill the message
				detection.pose.pose.position.x = platform_to_marker.x();
				detection.pose.pose.position.y = platform_to_marker.y();
				detection.pose.pose.position.z = 0.;
				angle = std::atan2(x_axis.y(), x_axis.x());
				detection.pose.pose.orientation.x = 0.;
				detection.pose.pose.orientation.y = 0.;
				detection.pose.pose.orientation.z = std::sin(angle/2.0);
				detection.pose.pose.orientation.w = std::cos(angle/2.0);
				detection.supports = 2;
				detection.points.clear();
				detection.points.push_back(__msg.detections[ii].pose.pose.position);
				detection.points.push_back(__msg.detections[jj].pose.pose.position);
				detection.baseline = actual_baseline;
				msg.detections.push_back(detection);
			}
		}
	}
	detector_publisher__.publish(msg);

}

void NodeBaselinePair::publishMarkers(const target_detector::Detections & __msg)
{

}

}
