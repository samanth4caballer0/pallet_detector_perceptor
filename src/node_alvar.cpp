#include "target_detector/node_alvar.h"

namespace TargetDetector
{

NodeAlvar::NodeAlvar()
{
	//
}

NodeAlvar::~NodeAlvar()
{
	//
}

bool NodeAlvar::init()
{
	// base init with common configs
	if ( !NodeBase::init() )
	{
		return false;
	}

	// specialized subscriber for this class
	alvar_markers_subscriber__ = nh__.subscribe("alvar_markers", 1, &NodeAlvar::alvarCallback, this);

	return true;
}

void NodeAlvar::alvarCallback(const ar_track_alvar_msgs::AlvarMarkers & __msg)
{
	// check if platform->sensor transform exists, and save it if not
	if ( !saveSensorTransform(__msg.header) )
	{
		ROS_WARN("NodeAlvar::alvarCallback(): missing tf from platform to camera.");
		return;
	}

	// Out message, keep the time stamp from the original header
	target_detector::Detections msg;
	msg.header = __msg.header;
	msg.header.frame_id = robot_frame__;
	msg.detections.resize(__msg.markers.size());

	// transform each alvar marker from camera frame to robot frame, and fill the out message
	Eigen::Quaterniond aux_qt;
	Eigen::Isometry3d T_camera_to_alvar; // alvar wrt camera (camera to alvar), from alvar detector
	Eigen::Isometry3d T_robot_to_alvar; // alvar wrt platform (platform to alvar), the one we want to compute
	double angle_z; // angle of alvar wrt the platform z axis (on the XY platform plane)
	for ( unsigned int ii = 0; ii < __msg.markers.size(); ii++ )
	{
		// build camera to alvar
		aux_qt.coeffs() <<	__msg.markers[ii].pose.pose.orientation.x,
							__msg.markers[ii].pose.pose.orientation.y,
							__msg.markers[ii].pose.pose.orientation.z,
							__msg.markers[ii].pose.pose.orientation.w;
		T_camera_to_alvar.linear() = aux_qt.matrix();
		T_camera_to_alvar.translation() <<	__msg.markers[ii].pose.pose.position.x,
											__msg.markers[ii].pose.pose.position.y,
											__msg.markers[ii].pose.pose.position.z;

		// compute robot to alvar
		T_robot_to_alvar = T_robot_to_sensor__[__msg.header.frame_id]*T_camera_to_alvar;

		// fill out message
		msg.detections[ii].id = __msg.markers[ii].id;
		msg.detections[ii].pose.pose.position.x = T_robot_to_alvar.translation().x();
		msg.detections[ii].pose.pose.position.y = T_robot_to_alvar.translation().y();
		msg.detections[ii].pose.pose.position.z = T_robot_to_alvar.translation().z();
		angle_z = std::atan2(T_robot_to_alvar.linear()(1,2),T_robot_to_alvar.linear()(0,2));
		msg.detections[ii].pose.pose.orientation.x = 0.0;
		msg.detections[ii].pose.pose.orientation.y = 0.0;
		msg.detections[ii].pose.pose.orientation.z = std::sin(angle_z/2.0);
		msg.detections[ii].pose.pose.orientation.w = std::cos(angle_z/2.0);
	}

	// publish
	detector_publisher__.publish(msg);
}

}
