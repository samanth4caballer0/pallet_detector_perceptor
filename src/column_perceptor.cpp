#include "column_perceptor.h"

namespace TargetDetector
{

bool ColumnPerceptor::init()
{
	if ( !configureParameters() )
	{
		ROS_ERROR("Failed to configure column detection");
		return false;
	}

	for ( int ii = 0; ii < lidars__.size(); ++ii )
		sensor_ids__[lidars__.at(ii)] = ii;

	initDetection();
	initDetections();

	tf_listener__.reset(new tf2_ros::TransformListener(tf_buffer__));

	detections_publisher__ = nh__.advertise<target_detector::Detections>("detections", 1, false);
	enable_server__ = nh__.advertiseService("enable", &ColumnPerceptor::enableCallback, this);

	detector__ = std::make_unique<Detectors::ColumnDetector>();
	if ( !detector__->configure(column_size__, max_detection_range__, column_isolation_distance__, override_support_points__) )
	{
		ROS_ERROR("Failed to configure column detector");
		return false;
	}

	ros::NodeHandle private_nh("~");
	private_nh.setParam("column_size", column_size__);
	private_nh.setParam("max_detection_range", max_detection_range__);
	private_nh.setParam("column_isolation_distance", column_isolation_distance__);
	private_nh.setParam("override_support_points", override_support_points__);
	private_nh.setParam("scan_decimation", decimation__);
	reconfigure_callback__ = boost::bind(&ColumnPerceptor::reconfigureCallback, this, _1, _2);
	reconfigure_server__.setCallback(reconfigure_callback__);

	if ( enabled__ )
		subscribeToLidars();

	if ( vizbose__ )
	{
		initMarker();
		markers_publisher__ = nh__.advertise<visualization_msgs::Marker>("visuals", 1, false);
	}

	return true;
}

void ColumnPerceptor::laserScanCallback(const sensor_msgs::LaserScanConstPtr & __scan_ptr, const std::string & __source_name)
{
	if ( !enabled__ )
		return;

	processScan(
		__scan_ptr->header,
		__source_name,
		__scan_ptr->angle_min,
		__scan_ptr->angle_max,
		__scan_ptr->ranges);
}

void ColumnPerceptor::processScan(
	const std_msgs::Header & __header,
	const std::string & __source_name,
	const double & __angle_min,
	const double & __angle_max,
	const std::vector<float> & __ranges)
{
	scan_decimation_counter__[__source_name]++;
	if ( scan_decimation_counter__[__source_name] % decimation__ != 0 )
		return;
	scan_decimation_counter__[__source_name] = 0;

	if ( !saveSensorTransform(__header) )
	{
		ROS_WARN("Failed to process scan because of missing tf");
		return;
	}

	std::lock_guard<std::mutex> lock(reconfigure_mutex__);
	const std::vector<Detectors::ColumnDetection> detected_columns = detector__->detect(__angle_min, __angle_max, __ranges);

	detections__.header = __header;
	detections__.header.frame_id = robot_frame__;
	detections__.detections.clear();
	detections__.source_name = __source_name;

	for ( const auto & detected_column : detected_columns )
	{
		geometry_msgs::Pose pose_in_sensor;
		pose_in_sensor.position.x = detected_column.centroid_x;
		pose_in_sensor.position.y = detected_column.centroid_y;
		pose_in_sensor.position.z = 0.0;
		pose_in_sensor.orientation.x = 0.0;
		pose_in_sensor.orientation.y = 0.0;
		pose_in_sensor.orientation.z = 0.0;
		pose_in_sensor.orientation.w = 1.0;

		geometry_msgs::Pose pose_in_robot;
		tf2::doTransform(pose_in_sensor, pose_in_robot, T_sensor_to_robot__[__header.frame_id]);
		detection__.pose.pose = pose_in_robot;

		for ( auto & covariance_value : detection__.pose.covariance )
			covariance_value = 0.0;

		Eigen::Matrix2d covariance_sensor = Eigen::Matrix2d::Zero();
		covariance_sensor(0,0) = detected_column.covariance_xx;
		covariance_sensor(1,1) = detected_column.covariance_yy;

		const Eigen::Isometry3d T_eigen = tf2::transformToEigen(T_sensor_to_robot__[__header.frame_id]);
		const Eigen::Matrix2d rotation_sensor_to_robot = T_eigen.rotation().block<2,2>(0,0);
		const Eigen::Matrix2d covariance_robot = rotation_sensor_to_robot * covariance_sensor * rotation_sensor_to_robot.transpose();

		detection__.pose.covariance[0] = covariance_robot(0,0);
		detection__.pose.covariance[1] = covariance_robot(0,1);
		detection__.pose.covariance[6] = covariance_robot(1,0);
		detection__.pose.covariance[7] = covariance_robot(1,1);

		detection__.intensity = 0.0;
		detection__.supports = detected_column.supports;
		detection__.radius = detected_column.nominal_radius;
		detection__.points.clear();
		detection__.baseline = 0.0;
		detections__.detections.push_back(detection__);
	}

	detections_publisher__.publish(detections__);
	if ( vizbose__ )
		publishMarkers(detections__, __source_name);
}

bool ColumnPerceptor::enableCallback(target_detector::DetectorEnable::Request & __request, target_detector::DetectorEnable::Response & __response)
{
	if ( !enabled__ && __request.enable )
		subscribeToLidars();

	if ( enabled__ && !__request.enable )
	{
		unsubscribeFromLidars();
		detections__.header.stamp = ros::Time::now();
		detections__.header.frame_id = robot_frame__;
		detections__.source_name.clear();
		detections__.detections.clear();
		detections_publisher__.publish(detections__);
	}

	enabled__ = __request.enable;
	__response.success = true;
	return true;
}

void ColumnPerceptor::reconfigureCallback(target_detector::column_perceptorConfig & __config, uint32_t __level)
{
	(void)__level;

	if ( first_dynamic_reconfigure__ )
	{
		first_dynamic_reconfigure__ = false;
		return;
	}

	ROS_INFO("Dynamic Reconfigure Request to Column Perceptor");
	std::lock_guard<std::mutex> lock(reconfigure_mutex__);

	decimation__ = __config.scan_decimation;
	override_support_points__ = __config.override_support_points;
	column_size__ = __config.column_size;
	max_detection_range__ = __config.max_detection_range;
	column_isolation_distance__ = __config.column_isolation_distance;

	if ( !detector__->configure(column_size__, max_detection_range__, column_isolation_distance__, override_support_points__) )
	{
		ROS_WARN("Rejected invalid dynamic reconfigure request for column detector");
		return;
	}

	detection__.radius = column_size__ / 2.0;
	if ( vizbose__ )
	{
		marker__.scale.x = column_size__;
		marker__.scale.y = column_size__;
		marker__.scale.z = column_size__;
	}
}

bool ColumnPerceptor::configureParameters()
{
	perceptor_name__ = ros::this_node::getNamespace().substr(ros::this_node::getNamespace().find_last_of('/') + 1);

	nh__.param("scan_decimation", decimation__, 1);
	nh__.param("column_isolation_distance", column_isolation_distance__, 0.0);
	nh__.param("override_support_points", override_support_points__, 0);

	if ( decimation__ < 1 )
	{
		ROS_ERROR("Scan decimation must be 1 or higher");
		return false;
	}

	return	getParamOrFail("enabled_by_default", enabled__) &&
			getParamOrFail("vizbose", vizbose__) &&
			getParamOrFail("lidars", lidars__) &&
			getParamOrFail("column_size", column_size__) &&
			getParamOrFail("max_detection_range", max_detection_range__) &&
			getParamOrFail("robot_frame", robot_frame__);
}

void ColumnPerceptor::subscribeToLidars()
{
	lidar_subscribers__.clear();
	for ( const auto & lidar : lidars__ )
	{
		scan_decimation_counter__[lidar] = 0;
		lidar_subscribers__.push_back(
			nh__.subscribe<sensor_msgs::LaserScan>(
				lidar,
				1,
				boost::bind(&ColumnPerceptor::laserScanCallback, this, _1, lidar)));
	}
}

void ColumnPerceptor::unsubscribeFromLidars()
{
	for ( auto & subscriber : lidar_subscribers__ )
		subscriber.shutdown();
	lidar_subscribers__.clear();
}

bool ColumnPerceptor::saveSensorTransform(const std_msgs::Header & __header)
{
	if ( !T_sensor_to_robot__.contains(__header.frame_id) )
	{
		try
		{
			geometry_msgs::TransformStamped T_sensor_to_robot;
			T_sensor_to_robot = tf_buffer__.lookupTransform(robot_frame__, __header.frame_id, __header.stamp, ros::Duration(1.0));
			T_sensor_to_robot__[__header.frame_id] = T_sensor_to_robot;
		}
		catch (tf2::TransformException & __ex)
		{
			ROS_WARN("%s", __ex.what());
			return false;
		}
	}
	return true;
}

void ColumnPerceptor::publishMarkers(const target_detector::Detections & __detections, const std::string & __source_name)
{
	if ( __detections.detections.empty() )
		return;

	marker__.header = __detections.header;
	marker__.id = sensor_ids__[__source_name];
	marker__.points.clear();
	for ( const auto & detection : __detections.detections )
		marker__.points.push_back(detection.pose.pose.position);
	markers_publisher__.publish(marker__);
}

}
