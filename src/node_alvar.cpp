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

	// visuals
	if ( vizbose__ )
	{
		viz_marker_publisher__ = nh__.advertise<visualization_msgs::MarkerArray>( "visuals", 1, true );
	}

	return true;
}

void NodeAlvar::alvarCallback(const ar_track_alvar_msgs::AlvarMarkers & __msg)
{
	// check if any alvar detection
	if ( __msg.markers.empty() )
		return;

	// check if platform->sensor transform exists, and save it if not
	if ( !saveSensorTransform(__msg.markers[0].header) )
	{
		ROS_WARN("NodeAlvar::alvarCallback(): missing tf from platform to camera.");
		return;
	}

	// Out message, keep the time stamp from the original header
	target_detector::Detections msg;
	msg.header = __msg.markers[0].header;
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
		T_camera_to_alvar.linear() = aux_qt.toRotationMatrix();
		T_camera_to_alvar.translation() <<	__msg.markers[ii].pose.pose.position.x,
											__msg.markers[ii].pose.pose.position.y,
											__msg.markers[ii].pose.pose.position.z;

		// debugging
		/*std::cout << "T_robot_to_sensor__: " << std::endl;
		std::cout << T_robot_to_sensor__[__msg.markers[0].header.frame_id].matrix() << std::endl;
		std::cout << "T_camera_to_alvar: " << std::endl;
		std::cout << T_camera_to_alvar.matrix() << std::endl;
		std::cout << "---------------" << std::endl;*/

		// compute robot to alvar
		T_robot_to_alvar = T_robot_to_sensor__[__msg.markers[0].header.frame_id]*T_camera_to_alvar;

		// fill out message
		msg.detections[ii].type = detector_type__;
		msg.detections[ii].id = __msg.markers[ii].id;
		msg.detections[ii].pose.pose.position.x = T_robot_to_alvar.translation().x();
		msg.detections[ii].pose.pose.position.y = T_robot_to_alvar.translation().y();
		msg.detections[ii].pose.pose.position.z = T_robot_to_alvar.translation().z();
		angle_z = std::atan2(T_robot_to_alvar.linear()(1,2),T_robot_to_alvar.linear()(0,2));
		msg.detections[ii].pose.pose.orientation.x = 0.0;
		msg.detections[ii].pose.pose.orientation.y = 0.0;
		msg.detections[ii].pose.pose.orientation.z = std::sin(angle_z/2.0);
		msg.detections[ii].pose.pose.orientation.w = std::cos(angle_z/2.0);
		msg.detections[ii].pose.covariance[0] = 0.2; // Cxx
		msg.detections[ii].pose.covariance[7] = 0.05; // Cyy
	}

	// publish
	detector_publisher__.publish(msg);

	/*msg.header = __msg.markers[0].header;
	for ( unsigned int ii = 0; ii < __msg.markers.size(); ii++ )
	{
		msg.detections[ii].pose.pose.position = __msg.markers[ii].pose.pose.position;
		msg.detections[ii].pose.pose.orientation = __msg.markers[ii].pose.pose.orientation;
	}*/
	if ( vizbose__ ) publishMarkers(msg);
}

void NodeAlvar::publishMarkers(const target_detector::Detections & __msg)
{
	visualization_msgs::MarkerArray msg;

	msg.markers.resize(__msg.detections.size()*2);
	for (unsigned int ii=0; ii<__msg.detections.size(); ii++)
	{
		msg.markers[2*ii].header = __msg.header; // keep the time stamp from the original header
		msg.markers[2*ii].id = 2*ii;
		msg.markers[2*ii].ns = "alvar_detections";
		msg.markers[2*ii].type = visualization_msgs::Marker::ARROW;
		msg.markers[2*ii].action = visualization_msgs::Marker::ADD;
		msg.markers[2*ii].lifetime = ros::Duration(0.5);
		msg.markers[2*ii].pose = __msg.detections[ii].pose.pose;
		msg.markers[2*ii].scale.x = 0.3;
		msg.markers[2*ii].scale.y = 0.05;
		msg.markers[2*ii].scale.z = 0.1;
		msg.markers[2*ii].color.r = 1.0;
		msg.markers[2*ii].color.g = 0.0;
		msg.markers[2*ii].color.b = 1.0;
		msg.markers[2*ii].color.a = 1.0;

		msg.markers[2*ii+1].header = __msg.header; // keep the time stamp from the original header
		msg.markers[2*ii+1].id = 2*ii+1;
		msg.markers[2*ii+1].ns = "alvar_detections";
		msg.markers[2*ii+1].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		msg.markers[2*ii+1].action = visualization_msgs::Marker::ADD;
		msg.markers[2*ii+1].lifetime = ros::Duration(0.5);
		msg.markers[2*ii+1].pose = __msg.detections[ii].pose.pose;
		msg.markers[2*ii+1].pose.position.x += 0.1;
		msg.markers[2*ii+1].scale.z = 0.4;
		msg.markers[2*ii+1].color.r = 1.0;
		msg.markers[2*ii+1].color.g = 1.0;
		msg.markers[2*ii+1].color.b = 1.0;
		msg.markers[2*ii+1].color.a = 1.0;
		msg.markers[2*ii+1].text = std::to_string(__msg.detections[ii].id);
	}

	// publish message
	viz_marker_publisher__.publish(msg);
}

}
