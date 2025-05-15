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

	//devel:  trying other ext calibration
	// T_robot_to_sensor__[__msg.markers[0].header.frame_id].translation() << 0.10, -0.02, 0.21;

	// Out message, keep the time stamp from the original header
	target_detector::Detections msg;
	msg.header = __msg.markers[0].header;
	msg.header.frame_id = robot_frame__;
	msg.detections.resize(__msg.markers.size());

	// transform each alvar marker from camera frame to robot frame, and fill the out message
	double range; // range in the camera xy plane, assumes camera x pointing forward and y pointing left
	double bearing; // bearing in the camera xy plane, assumes camera x pointing forward and y pointing left
	Eigen::Matrix2d Crb; // Covariance in range-bearing space
	Eigen::Matrix2d Jrb; // Jacobian wrt range and bearing
	Eigen::Matrix2d Cxy_camera; // Covariance in xy space wrt camera
	Eigen::Matrix2d Cxy_robot; // Covariance in xy space wrt robot
	Eigen::Matrix2d Rrobot_to_camera; //2D rotation of robot to camera (camera wrt robot)
	Eigen::Quaterniond aux_qt;
	Eigen::Isometry3d T_camera_to_alvar; // alvar wrt camera (camera to alvar), from alvar detector
	Eigen::Isometry3d T_robot_to_alvar; // alvar wrt platform (platform to alvar), the one we want to compute
	double angle_z; // angle of alvar wrt the platform z axis (on the XY platform plane)
	for ( unsigned int ii = 0; ii < __msg.markers.size(); ii++ )
	{
		// 1. compute range in camera xy plane
		range = std::sqrt( std::pow(__msg.markers[ii].pose.pose.position.x,2) + std::pow(__msg.markers[ii].pose.pose.position.y,2) );
		//std::cout << "range: " << range << std::endl;

		// 2. compute bearing in camera xy plane
		bearing = 2*std::atan2(__msg.markers[ii].pose.pose.position.y,__msg.markers[ii].pose.pose.position.x);
		//std::cout << "bearing: " << bearing << std::endl;

		// 3. Set the covariance in camera range-bearing space
		Crb <<
			0.05*0.05*std::pow(range,4), 0., // std_dev = 0.05r^2
			0., std::pow(2*M_PI/180.,2); // std dev 2 deg

		// 4. Compute the Jacobian wrt range and bearing
		Jrb <<
			std::cos(bearing), -range*std::sin(bearing),
			std::sin(bearing),  range*std::cos(bearing);

		// 5. Compute the covariance in camera xy space
		Cxy_camera = Jrb*Crb*Jrb.transpose();

		// 6. Compute the covariance in platform xy space
		Rrobot_to_camera = T_robot_to_sensor__[__msg.markers[0].header.frame_id].matrix().block<2,2>(0,0);
		Cxy_robot = Rrobot_to_camera.inverse()*Cxy_camera*Rrobot_to_camera.inverse().transpose();
		//std::cout << "T_robot_to_sensor__:" << std::endl << T_robot_to_sensor__[__msg.markers[0].header.frame_id].matrix() << std::endl;
		//std::cout << "Rrobot_to_camera: " << std::endl << Rrobot_to_camera << std::endl << "--------------" << std::endl;
		//std::cout << "Cxy_robot: " << std::endl << Cxy_robot << std::endl << "--------------" << std::endl;

		// 7. Transform the detection expressed in sensor frame to be expressed in robot frame
		aux_qt.coeffs() <<	__msg.markers[ii].pose.pose.orientation.x,
							__msg.markers[ii].pose.pose.orientation.y,
							__msg.markers[ii].pose.pose.orientation.z,
							__msg.markers[ii].pose.pose.orientation.w;
		T_camera_to_alvar.linear() = aux_qt.toRotationMatrix();
		T_camera_to_alvar.translation() <<	__msg.markers[ii].pose.pose.position.x,
											__msg.markers[ii].pose.pose.position.y,
											__msg.markers[ii].pose.pose.position.z;
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
		msg.detections[ii].pose.covariance[0] = Cxy_robot(0,0); // Cxx
		msg.detections[ii].pose.covariance[1] = Cxy_robot(0,1); // Cxy
		msg.detections[ii].pose.covariance[6] = Cxy_robot(1,0); // Cyx
		msg.detections[ii].pose.covariance[7] = Cxy_robot(1,1); // Cyy
	}

	// publish
	detector_publisher__.publish(msg);
	if ( vizbose__ ) publishMarkers(msg);
}

void NodeAlvar::publishMarkers(const target_detector::Detections & __msg)
{
	visualization_msgs::MarkerArray msg;
	Eigen::Matrix2d Cxy;
	double lambda_1, lambda_2, lambda_max, lambda_min, angle;

	msg.markers.resize(__msg.detections.size()*3);
	for (unsigned int ii=0; ii<__msg.detections.size(); ii++)
	{
		msg.markers[3*ii].header = __msg.header; // keep the time stamp from the original header
		//msg.markers[3*ii].header.frame_id = robot_frame__;
		msg.markers[3*ii].id = 3*ii;
		msg.markers[3*ii].ns = "alvar_detections";
		msg.markers[3*ii].type = visualization_msgs::Marker::ARROW;
		msg.markers[3*ii].action = visualization_msgs::Marker::ADD;
		msg.markers[3*ii].lifetime = ros::Duration(0.5);
		msg.markers[3*ii].pose = __msg.detections[ii].pose.pose;
		msg.markers[3*ii].scale.x = 0.3;
		msg.markers[3*ii].scale.y = 0.03;
		msg.markers[3*ii].scale.z = 0.06;
		msg.markers[3*ii].color.r = 1.0;
		msg.markers[3*ii].color.g = 0.0;
		msg.markers[3*ii].color.b = 1.0;
		msg.markers[3*ii].color.a = 0.5;

		msg.markers[3*ii+1].header = __msg.header; // keep the time stamp from the original header
		//msg.markers[3*ii+1].header.frame_id = robot_frame__;
		msg.markers[3*ii+1].id = 3*ii+1;
		msg.markers[3*ii+1].ns = "alvar_detections";
		msg.markers[3*ii+1].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		msg.markers[3*ii+1].action = visualization_msgs::Marker::ADD;
		msg.markers[3*ii+1].lifetime = ros::Duration(0.5);
		msg.markers[3*ii+1].pose = __msg.detections[ii].pose.pose;
		msg.markers[3*ii+1].pose.position.x += 0.1;
		msg.markers[3*ii+1].scale.z = 0.4;
		msg.markers[3*ii+1].color.r = 1.0;
		msg.markers[3*ii+1].color.g = 1.0;
		msg.markers[3*ii+1].color.b = 1.0;
		msg.markers[3*ii+1].color.a = 1.0;
		msg.markers[3*ii+1].text = std::to_string(__msg.detections[ii].id);

		msg.markers[3*ii+2].header = __msg.header; // keep the time stamp from the original header
		//msg.markers[3*ii+2].header.frame_id = robot_frame__;
		msg.markers[3*ii+2].id = 3*ii+2;
		msg.markers[3*ii+2].ns = "alvar_detections";
		msg.markers[3*ii+2].type = visualization_msgs::Marker::SPHERE;
		msg.markers[3*ii+2].action = visualization_msgs::Marker::ADD;
		msg.markers[3*ii+2].lifetime = ros::Duration(0.5);
		msg.markers[3*ii+2].pose.position = __msg.detections[ii].pose.pose.position;
		Cxy <<
			__msg.detections[ii].pose.covariance[0], __msg.detections[ii].pose.covariance[1],
			__msg.detections[ii].pose.covariance[6], __msg.detections[ii].pose.covariance[7];
		lambda_1 = (Cxy(0,0)+Cxy(1,1)) + std::sqrt( std::pow( (Cxy(0,0)-Cxy(1,1)),2 ) + 4*std::pow( Cxy(1,0), 2) );
		lambda_2 = (Cxy(0,0)+Cxy(1,1)) - std::sqrt( std::pow( (Cxy(0,0)-Cxy(1,1)),2 ) + 4*std::pow( Cxy(1,0), 2) );
		if (lambda_1 < 0) std::cout << "lambda_1: " << lambda_1 << std::endl;
		if (lambda_2 < 0) std::cout << "lambda_2: " << lambda_2 << std::endl;
		lambda_max = 2*std::sqrt(std::max(lambda_1, lambda_2));
		lambda_min = 2*std::sqrt(std::min(lambda_1, lambda_2));
		angle = 0.25*std::atan2(2*Cxy(1,0), Cxy(0,0)-Cxy(1,1));
		msg.markers[3*ii+2].pose.orientation.x = 0.;
		msg.markers[3*ii+2].pose.orientation.y = 0.;
		msg.markers[3*ii+2].pose.orientation.z = std::sin(angle/2.);
		msg.markers[3*ii+2].pose.orientation.w = std::cos(angle/2.);
		msg.markers[3*ii+2].scale.x = lambda_max;
		msg.markers[3*ii+2].scale.y = lambda_min;
		msg.markers[3*ii+2].scale.z = 0.01;
		msg.markers[3*ii+2].color.r = 1.0;
		msg.markers[3*ii+2].color.g = 1.0;
		msg.markers[3*ii+2].color.b = 0.0;
		msg.markers[3*ii+2].color.a = 0.3;

	}

	// publish message
	viz_marker_publisher__.publish(msg);
}

}
