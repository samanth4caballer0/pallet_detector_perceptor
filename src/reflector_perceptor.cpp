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

	if ( decimation__ < 1)
	{
		ROS_ERROR("Scan decimation must be 1 or higher");
		return false;
	}

	// update stuff that depends on given parameters
	for ( int i = 0; i < lidars__.size(); i++) // this is just to give a unique id to each sensor, being the id an int (for markers publishing)
		sensor_ids__[lidars__.at(i)] = i;

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

void ReflectorPerceptor::laserScanCallback(const sensor_msgs::LaserScanConstPtr & __scan_ptr, const std::string & __source_name)
{
	if (!enabled__)
		return;

	processScan(
		__scan_ptr->header,
		__source_name,
		__scan_ptr->angle_min,
		__scan_ptr->angle_max,
		__scan_ptr->ranges,
		__scan_ptr->intensities,
		nullptr);
}

void ReflectorPerceptor::laserScanExtendedCallback(const sick_safetyscanners::ExtendedLaserScanMsgConstPtr & __scan_ptr, const std::string & __source_name)
{
	if ( !enabled__ )
		return;
	processScan(
		__scan_ptr->laser_scan.header,
		__source_name,
		__scan_ptr->laser_scan.angle_min,
		__scan_ptr->laser_scan.angle_max,
		__scan_ptr->laser_scan.ranges,
		__scan_ptr->laser_scan.intensities,
		&__scan_ptr->reflektor_status);
}

void ReflectorPerceptor::processScan(
	const std_msgs::Header & __header,
	const std::string & __source_name,
	double __angle_min,
	double __angle_max,
	const std::vector<float> & __ranges,
	const std::vector<float> & __intensities,
	const std::vector<uint8_t> * __reflector_hits)
{
	// apply decimation
	scan_decimation_counter__[__source_name]++;
	if (scan_decimation_counter__[__source_name] % decimation__ != 0)
		return;
	scan_decimation_counter__[__source_name] = 0;

	// add platform->lidar transform if not already available
	if (!saveSensorTransform(__header))
	{
		ROS_WARN("Failed to process scan because of missing tf");
		return;
	}

	// detect and generate detections
	std::vector<Detectors::ReflectorDetection> detected_reflectors;

	if (__reflector_hits)
	{
		detected_reflectors = detector__->detect(
			__angle_min, __angle_max,
			__ranges, __intensities,
			*__reflector_hits,
			T_robot_to_sensor_2d__[__header.frame_id]);
	}
	else
	{
		detected_reflectors = detector__->detect(
			__angle_min, __angle_max,
			__ranges, __intensities,
			T_robot_to_sensor_2d__[__header.frame_id]);
	}

	// fill detections msg
	detections__.header = __header;
	detections__.header.frame_id = robot_frame__;
	detections__.detections.clear();
	detections__.source_name = __source_name;

	for (auto & detected_reflector : detected_reflectors)
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
	if (vizbose__)
		publishMarkers(detections__, __source_name);
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

	// parse optional parameters
	if( !getParamOrFail("scan_type", scan_type__) )
		return false;
	
	if( scan_type__ != "sensor_msgs/LaserScan" && 
		scan_type__ != "sick_safetyscanners/ExtendedLaserScanMsg")
	{
		ROS_ERROR("Unknown scan_type");
		return false;	
	}

	if (scan_type__ == "sensor_msgs/LaserScan")
	{
		if (!getParamOrFail("min_reflector_intensity", min_reflector_intensity__))
			return false;
	}

	// common parameters
	return	getParamOrFail("enabled_by_default", enabled__) &&
			getParamOrFail("vizbose", vizbose__) &&
			getParamOrFail("lidars", lidars__) &&
			getParamOrFail("reflector_size", reflector_size__) &&
			getParamOrFail("max_detection_range", max_detection_range__) &&
			getParamOrFail("robot_frame", robot_frame__) &&
			getParamOrFail("scan_decimation", decimation__);
}

void ReflectorPerceptor::subscribeToLidars()
{
	lidar_subscribers__.clear();
	for ( auto & lidar : lidars__ )
	{
		scan_decimation_counter__[lidar] = 0;
		if ( scan_type__ == "sensor_msgs/LaserScan" )
		{	
			lidar_subscribers__.push_back(
				nh__.subscribe<sensor_msgs::LaserScan>(lidar, 1,
					boost::bind(&ReflectorPerceptor::laserScanCallback, this, _1, lidar)));
		}
		else if ( scan_type__ == "sick_safetyscanners/ExtendedLaserScanMsg" )
		{
			lidar_subscribers__.push_back(
				nh__.subscribe<sick_safetyscanners::ExtendedLaserScanMsg>(lidar, 1,
					boost::bind(&ReflectorPerceptor::laserScanExtendedCallback, this, _1, lidar)));
		}
		else
		{
			ROS_ERROR("Unable to subscribe to lidars. Unknown scan_type");
		}
	}
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

void ReflectorPerceptor::publishMarkers(const target_detector::Detections & __detections, const std::string & __source_name)
{
	// only publish if not emtpy, otherwise rviz generates error
	if ( __detections.detections.empty() )
		return;

	marker__.header = __detections.header;
	marker__.id = sensor_ids__[__source_name];

	marker__.points.clear();
	for ( auto & detection : __detections.detections )
		marker__.points.push_back(detection.pose.pose.position);

	markers_publisher__.publish(marker__);
}

}
