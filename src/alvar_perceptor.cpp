#include "alvar_perceptor.h"

namespace TargetDetector
{

bool AlvarPerceptor::init()
{

	if ( !configureParameters() )
	{
		ROS_ERROR("Failed to configure alvar detection");
		return false;
	}

	initDetection();

	enable_bundle_detection_publisher__ = nh__.advertise<std_msgs::Bool>("bundle_enable_detection", 1, false);

	if ( enabled__ )
	{
		subscribeToData();
		enableBundleDetection(true);
	}
	else
	{
		enableBundleDetection(false);
	}

	if ( vizbose__ )
	{
		initMarker();
		markers_publisher__ = nh__.advertise<visualization_msgs::MarkerArray>("visuals", 1, false);
	}

	tf_listener__.reset(new tf2_ros::TransformListener(tf_buffer__));

	detections_out_publisher__ = nh__.advertise<target_detector::Detections>("detections", 1, false);
	enable_server__ = nh__.advertiseService("enable", &AlvarPerceptor::enableCallback, this);

	return true;
};

void AlvarPerceptor::detectionsInCallback(const ar_track_alvar_msgs::AlvarMarkers & __bundles)
{
	if ( !enabled__ )
		return;

	// check if any alvar detection
	if ( __bundles.markers.empty() )
		return;

	// check if platform->sensor transform exists, and save it if not
	if ( !saveSensorTransform(__bundles.markers.front().header) )
	{
		ROS_WARN("Failed to process scan because of missing tf to camera");
		return;
	}

	// Out message, keep the time stamp from the original header
	target_detector::Detections detections;
	detections.header = __bundles.markers.front().header;
	detections.header.frame_id = robot_frame__;

	// transform each alvar marker from camera frame to robot frame
	double range; // range in the camera xy plane, assumes camera x pointing forward and y pointing left
	double bearing; // bearing in the camera xy plane, assumes camera x pointing forward and y pointing left
	Eigen::Matrix2d covariance_range_bearing; // Covariance in range-bearing space
	Eigen::Matrix2d jacobian_range_bearing; // Jacobian wrt range and bearing
	Eigen::Matrix2d Covariance_xy_camera; // Covariance in xy space wrt camera
	Eigen::Matrix2d Covariance_xy_robot; // Covariance in xy space wrt robot
	Eigen::Matrix2d Rotation_robot_to_camera; //2D rotation of robot to camera (camera wrt robot)
	Eigen::Quaterniond auxiliar_quaternion;
	Eigen::Isometry3d T_camera_to_alvar; // alvar wrt camera (camera to alvar), from alvar detector
	Eigen::Isometry3d T_robot_to_alvar; // alvar wrt platform (platform to alvar), the one we want to compute
	double angle_z; // angle of alvar wrt the platform z axis (on the XY platform plane)
	for ( auto & bundle : __bundles.markers )
	{
		// 0. Check if corresponds to the desired id
		if ( alvar_id__ >= 0) // if we look for a specific id (-1 means all)
			if ( bundle.id != alvar_id__ ) // skip if it does not match
				continue;

		// 1. compute range in camera xy plane
		range = std::sqrt( std::pow(bundle.pose.pose.position.x, 2) + std::pow(bundle.pose.pose.position.y, 2) );
		//std::cout << "range: " << range << std::endl;

		// 2. compute bearing in camera xy plane
		bearing = 2*std::atan2(bundle.pose.pose.position.y, bundle.pose.pose.position.x);
		//std::cout << "bearing: " << bearing << std::endl;

		// 3. Set the covariance in camera range-bearing space
		covariance_range_bearing <<
			0.02*0.02 + 0.05*0.05*std::pow(range, 4), 0.0, // var = 0.02^2+0.05^2r^4
			0.0, std::pow(2*M_PI/180.0, 2); // std dev 2 deg

		// 4. Compute the Jacobian wrt range and bearing
		jacobian_range_bearing <<
			std::cos(bearing), -range*std::sin(bearing),
			std::sin(bearing),  range*std::cos(bearing);

		// 5. Compute the covariance in camera xy space
		Covariance_xy_camera = jacobian_range_bearing*covariance_range_bearing*jacobian_range_bearing.transpose();

		// 6. Compute the covariance in platform xy space
		Rotation_robot_to_camera = T_robot_to_sensor__[__bundles.markers.front().header.frame_id].matrix().block<2,2>(0,0);
		Covariance_xy_robot = Rotation_robot_to_camera.inverse()*Covariance_xy_camera*Rotation_robot_to_camera.inverse().transpose();

		// 7. Transform the detection expressed in sensor frame to be expressed in robot frame
		// Looking at alvar marker, marker frame is: X to the right, Y up, Z towards the viewer
		// target_detector convention is: X towards the viewer, Y to the right,, Z up
		auxiliar_quaternion.coeffs() <<	bundle.pose.pose.orientation.x,
							bundle.pose.pose.orientation.y,
							bundle.pose.pose.orientation.z,
							bundle.pose.pose.orientation.w;
		T_camera_to_alvar.linear() = auxiliar_quaternion.toRotationMatrix();
		T_camera_to_alvar.translation() <<	bundle.pose.pose.position.x,
											bundle.pose.pose.position.y,
											bundle.pose.pose.position.z;
		T_robot_to_alvar = T_robot_to_sensor__[__bundles.markers.front().header.frame_id]*T_camera_to_alvar;
		//angle_z = std::atan2(T_robot_to_alvar.linear()(1,2),T_robot_to_alvar.linear()(0,2));
		angle_z = std::atan2(-T_robot_to_alvar.linear()(1,1),-T_robot_to_alvar.linear()(0,1));

		// fill out message
		detection__.id = bundle.id;
		detection__.pose.pose.position.x = T_robot_to_alvar.translation().x();
		detection__.pose.pose.position.y = T_robot_to_alvar.translation().y();
		detection__.pose.pose.position.z = T_robot_to_alvar.translation().z();
		detection__.pose.pose.orientation.z = std::sin(angle_z/2.0);
		detection__.pose.pose.orientation.w = std::cos(angle_z/2.0);
		detection__.pose.covariance[0] = Covariance_xy_robot(0,0); // Cxx
		detection__.pose.covariance[1] = Covariance_xy_robot(0,1); // Cxy
		detection__.pose.covariance[6] = Covariance_xy_robot(1,0); // Cyx
		detection__.pose.covariance[7] = Covariance_xy_robot(1,1); // Cyy

		detections.detections.push_back(detection__);
	}

	detections_out_publisher__.publish(detections);
	
	if ( vizbose__ )
		publishMarkers(detections);
}

bool AlvarPerceptor::enableCallback(target_detector::DetectorEnable::Request & __request, target_detector::DetectorEnable::Response & __response)
{
	// we update id, even if already enabled
	if ( __request.enable )
		alvar_id__ = __request.alvar_marker_id;

	if ( !enabled__ && __request.enable )
	{
		subscribeToData();
		enableBundleDetection(true);
	}

	if ( enabled__ && !__request.enable )
	{
		unsubscribeFromData();
		enableBundleDetection(false);
	}

	enabled__ = __request.enable;
	__response.success = true;
	return true;
}

bool AlvarPerceptor::configureParameters()
{
	perceptor_name__ = ros::this_node::getNamespace().substr(ros::this_node::getNamespace().find_last_of('/') + 1);
	return	getParamOrFail("enabled_by_default", enabled__) &&
			getParamOrFail("robot_frame", robot_frame__) &&
			getParamOrFail("marker_size", marker_size__) &&
			getParamOrFail("vizbose", vizbose__);
}

void AlvarPerceptor::subscribeToData()
{
	detections_in_subscriber__ = nh__.subscribe("detections_in", 1, &AlvarPerceptor::detectionsInCallback, this);
}

void AlvarPerceptor::unsubscribeFromData()
{
	detections_in_subscriber__.shutdown();
}

void AlvarPerceptor::publishMarkers(const target_detector::Detections & __detections)
{
	// only publish if not emtpy, otherwise rviz generates error
	if ( __detections.detections.empty() )
		return;

	marker__.header = __detections.header;
	marker__.points.clear();

	visualization_msgs::MarkerArray marker_array;

	for ( auto & detection : __detections.detections )
	{
		marker__.id = detection.id;
		marker__.pose = detection.pose.pose;
		marker_array.markers.push_back(marker__);
	}

	markers_publisher__.publish(marker_array);
}

bool AlvarPerceptor::saveSensorTransform(const std_msgs::Header & __header)
{
	if ( !T_robot_to_sensor__.contains(__header.frame_id) )
	{
		geometry_msgs::TransformStamped T_robot_sensor; // from robot to sensor
		Eigen::Quaterniond aux_qt;
		double angle_z;

		try
		{
			// get _transform from tf
			T_robot_sensor = tf_buffer__.lookupTransform(robot_frame__, __header.frame_id, __header.stamp, ros::Duration(1.0));

			// convert to Eigen Isometry3d
			aux_qt.coeffs() <<
				T_robot_sensor.transform.rotation.x,
				T_robot_sensor.transform.rotation.y,
				T_robot_sensor.transform.rotation.z,
				T_robot_sensor.transform.rotation.w;
			T_robot_to_sensor__[__header.frame_id].linear() = aux_qt.matrix();
			T_robot_to_sensor__[__header.frame_id].translation() <<
				T_robot_sensor.transform.translation.x,
				T_robot_sensor.transform.translation.y,
				T_robot_sensor.transform.translation.z;

			// convert to Eigen::Isometry2_d_
			angle_z = 2.0*std::atan2(T_robot_sensor.transform.rotation.z, T_robot_sensor.transform.rotation.w);
			T_robot_to_sensor_2d__[__header.frame_id].matrix() <<
				std::cos(angle_z), -std::sin(angle_z), T_robot_sensor.transform.translation.x,
				std::sin(angle_z),  std::cos(angle_z), T_robot_sensor.transform.translation.y,
				0,0,1;
		}
		catch (tf2::TransformException & __ex)
		{
			ROS_WARN("%s", __ex.what());
			ros::Duration(1.0).sleep();
			return false;
		}
	}

	return true;
}

}