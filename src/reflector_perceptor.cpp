#include "reflector_perceptor.h"

namespace TargetDetector
{

bool ReflectorPerceptor::init()
{
	if ( !configureParameters() )
	{
		ROS_ERROR("Failed to configure reflector detection");
		return false;
	}

	initDetection();
	initAllDetections();

	tf_listener__.reset(new tf2_ros::TransformListener(tf_buffer__));

	detections_publisher__ = nh__.advertise<target_detector::Detections>("detections", 1, false);
	enable_server__ = nh__.advertiseService("enable", &ReflectorPerceptor::enableCallback, this);

	detector__ = std::make_unique<Detectors::ReflectorDetector>();
	detector__->configure(reflector_size__, min_reflector_intensity__, max_detection_range__);

	detections_timer__ =  nh__.createTimer(ros::Duration(1.0/rate__), &ReflectorPerceptor::detectionsTimerCallback, this, false, false);

	if ( enabled__ )
	{
		subscribeToLidars();
		detections_timer__.start();
	}

	if ( vizbose__ )
	{
		initMarker();
		markers_publisher__ = nh__.advertise<visualization_msgs::Marker>("visuals", 1, false);
	}

	return true;
};

void ReflectorPerceptor::laserScanCallback(const sensor_msgs::LaserScanConstPtr & __scan_ptr)
{
	if ( !enabled__ )
		return;

	// add platform->lidar transform if not already available
	if ( !saveSensorTransform(__scan_ptr->header) )
	{
		ROS_WARN("Failed to process scan because of missing tf");
		return;
	}

	std::vector<Detectors::ReflectorDetection> detected_reflectors = detector__->detect(__scan_ptr->angle_min, __scan_ptr->angle_max,
		__scan_ptr->ranges, __scan_ptr->intensities, T_robot_to_sensor_2d__[__scan_ptr->header.frame_id]);

	target_detector::Detections detections;
	detections.header = __scan_ptr->header;
	detections.header.frame_id = robot_frame__;
	for ( auto & detected_reflector : detected_reflectors )
	{
		detection__.pose.pose.position.x = detected_reflector.centroid_x;
		detection__.pose.pose.position.y = detected_reflector.centroid_y;
		detection__.pose.covariance[0] = detected_reflector.covariance_xx;
		detection__.pose.covariance[7] = detected_reflector.covariance_yy;
		detection__.intensity = detected_reflector.intensity;
		detection__.supports = detected_reflector.supports;
		detections.detections.push_back(detection__);
	}

	last_detections__[__scan_ptr->header.frame_id] = detections;
}

void ReflectorPerceptor::detectionsTimerCallback(const ros::TimerEvent & __timer_event)
{
	all_detections__.header.stamp = ros::Time::now();
	all_detections__.detections.clear();
	for ( auto & [lidar, detections] : last_detections__ )
	{
		if ( ros::Time::now() - detections.header.stamp > max_detection_age__ )
		{
			// forget detections that are too old now
			detections.detections.clear();
			continue;
		}

		all_detections__.detections.insert(
			all_detections__.detections.end(),
			detections.detections.begin(),
			detections.detections.end()
		);
	}
	detections_publisher__.publish(all_detections__);

	if ( vizbose__ )
		publishMarkers(all_detections__);
}

bool ReflectorPerceptor::enableCallback(target_detector::DetectorEnable::Request & __request, target_detector::DetectorEnable::Response & __response)
{
	if ( !enabled__ && __request.enable )
	{
		subscribeToLidars();
		detections_timer__.start();
	}

	if ( enabled__ && !__request.enable )
	{
		unsubscribeFromLidars();
		detections_timer__.stop();

		// since we disable, make sure to publish an empty detections message to clear interested parties
		all_detections__.header.stamp = ros::Time::now();
		all_detections__.detections.clear();
		detections_publisher__.publish(all_detections__);
	}

	enabled__ = __request.enable;
	__response.success = true;
	return true;
}

bool ReflectorPerceptor::configureParameters()
{
	perceptor_name__ = ros::this_node::getNamespace().substr(ros::this_node::getNamespace().find_last_of('/') + 1);
	return	getParamOrFail("enabled_by_default", enabled__) &&
			getParamOrFail("vizbose", vizbose__) &&
			getParamOrFail("lidars", lidars__) &&
			getParamOrFail("reflector_size", reflector_size__) &&
			getParamOrFail("min_reflector_intensity", min_reflector_intensity__) &&
			getParamOrFail("max_detection_range", max_detection_range__) &&
			getParamOrFail("robot_frame", robot_frame__) &&
			getParamOrFail("rate", rate__);
}

void ReflectorPerceptor::subscribeToLidars()
{
	lidar_subscribers__.clear();
	for ( auto & lidar : lidars__ )
		lidar_subscribers__.push_back(nh__.subscribe(lidar, 1, &ReflectorPerceptor::laserScanCallback, this));
}

void ReflectorPerceptor::unsubscribeFromLidars()
{
	for ( auto & subscriber : lidar_subscribers__ )
		subscriber.shutdown();
	lidar_subscribers__.clear();
}

bool ReflectorPerceptor::saveSensorTransform(const std_msgs::Header & __header)
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

void ReflectorPerceptor::publishMarkers(const target_detector::Detections & __detections)
{
	// only publish if not emtpy, otherwise rviz generates error
	if ( __detections.detections.empty() )
		return;

	marker__.header = __detections.header;

	marker__.points.clear();
	for ( auto & detection : __detections.detections )
		marker__.points.push_back(detection.pose.pose.position);

	markers_publisher__.publish(marker__);
}

}