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

	// update stuff that depends on given parameters
	for ( int i = 0; i < lidars__.size(); i++) // this is just to give a unique id to each sensor, being the id an int (for markers publishing)
	{
		sensor_ids__[lidars__.at(i)] = i;
		processing_period__[lidars__.at(i)] = ros::Duration(1.0/rate__);
		last_processed_scan_stamp__[lidars__.at(i)] = ros::Time(0);
	}

	initDetection();
	initDetections();

	tf_listener__.reset(new tf2_ros::TransformListener(tf_buffer__));

	detections_publisher__ = nh__.advertise<target_detector::Detections>("detections", 1, false);
	enable_server__ = nh__.advertiseService("enable", &ReflectorPerceptor::enableCallback, this);

	detector__ = std::make_unique<Detectors::ReflectorDetector>();
	detector__->configure(reflector_size__, min_reflector_intensity__, max_detection_range__);

	if ( enabled__ )
		subscribeToLidars();

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

	std::string sensor_name = __scan_ptr->header.frame_id;

	// check if not enough time has passed to allow processing a scan again (compares using scan stamps)
	if ( __scan_ptr->header.stamp - last_processed_scan_stamp__[sensor_name] < processing_period__[sensor_name] )
		return;

	last_processed_scan_stamp__[sensor_name] = __scan_ptr->header.stamp;

	// add platform->lidar transform if not already available
	if ( !saveSensorTransform(__scan_ptr->header) )
	{
		ROS_WARN("Failed to process scan because of missing tf");
		return;
	}

	// detect and generate detections
	std::vector<Detectors::ReflectorDetection> detected_reflectors = detector__->detect(__scan_ptr->angle_min, __scan_ptr->angle_max,
		__scan_ptr->ranges, __scan_ptr->intensities, T_robot_to_sensor_2d__[__scan_ptr->header.frame_id]);

	detections__.header = __scan_ptr->header;
	detections__.header.frame_id = robot_frame__;
	detections__.detections.clear();
	for ( auto & detected_reflector : detected_reflectors )
	{
		detection__.pose.pose.position.x = detected_reflector.centroid_x;
		detection__.pose.pose.position.y = detected_reflector.centroid_y;
		detection__.pose.covariance[0] = detected_reflector.covariance_xx;
		detection__.pose.covariance[7] = detected_reflector.covariance_yy;
		detection__.intensity = detected_reflector.intensity;
		detection__.supports = detected_reflector.supports;
		detections__.detections.push_back(detection__);
	}

	// publish current detections
	detections_publisher__.publish(detections__);
	if ( vizbose__ )
		publishMarkers(detections__, __scan_ptr->header.frame_id);
}

bool ReflectorPerceptor::enableCallback(target_detector::DetectorEnable::Request & __request, target_detector::DetectorEnable::Response & __response)
{
	if ( !enabled__ && __request.enable )
		subscribeToLidars();

	if ( enabled__ && !__request.enable )
	{
		unsubscribeFromLidars();

		// since we disable, make sure to publish an empty detections message to clear interested parties
		detections__.header.stamp = ros::Time::now();
		detections__.detections.clear();
		detections_publisher__.publish(detections__);
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

void ReflectorPerceptor::publishMarkers(const target_detector::Detections & __detections, const std::string & __sensor_name)
{
	// only publish if not emtpy, otherwise rviz generates error
	if ( __detections.detections.empty() )
		return;

	marker__.header = __detections.header;
	marker__.id = sensor_ids__[__sensor_name];

	marker__.points.clear();
	for ( auto & detection : __detections.detections )
		marker__.points.push_back(detection.pose.pose.position);

	markers_publisher__.publish(marker__);
}

}