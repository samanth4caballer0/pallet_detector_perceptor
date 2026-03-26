#include "tmk_uwb_perceptor.h"

namespace TargetDetector
{

bool TmkUwbPerceptor::init()
{

	if ( !configureParameters() )
	{
		ROS_ERROR("Failed to configure tmk_uwb detection");
		return false;
	}

	initDetection();
	initDetections();

	if ( enabled__ )
		subscribeToData();

	if ( vizbose__ )
	{
		initMarker();
		markers_publisher__ = nh__.advertise<visualization_msgs::Marker>("visuals", 1, false);
	}

	tf_listener__.reset(new tf2_ros::TransformListener(tf_buffer__));

	detections_out_publisher__ = nh__.advertise<target_detector::Detections>("detections", 1, false);
	if ( publish_sensor_frame_detections__ )
		detections_sensor_frame_publisher__ = nh__.advertise<target_detector::Detections>("detections_sensor_frame", 1, false);
	enable_server__ = nh__.advertiseService("enable", &TmkUwbPerceptor::enableCallback, this);

	return true;
};

void TmkUwbPerceptor::detectionsInCallback(const tmk_uwb::UwbMeasurement & __msg)
{
	if ( !enabled__ )
		return;


	// check if platform->sensor transform exists, and save it if not
	if ( !saveSensorTransform(__msg.header) )
	{
		ROS_WARN("Failed to process uwb because of missing tf to sensor");
		return;
	}

	// Out message, keep the time stamp from the original header
	target_detector::Detections detections_in_sensor;
	detections_in_sensor.header = __msg.header;
	detections_in_sensor.source_name = "tmk_uwb_" + std::to_string(__msg.anchor_id);

	target_detector::Detections detections_in_robot;
	detections_in_robot.header = __msg.header;
	detections_in_robot.header.frame_id = robot_frame__;
	detections_in_robot.source_name = detections_in_sensor.source_name;

	// Transform each tmk_uwb measurement from sensor frame to robot frame
	// Assumes radio beacons both onboard and landmarks mounted with z axis pointing down, so invert the sign at azimuth and cp_azimuth
	geometry_msgs::Pose pose_in_sensor;
	pose_in_sensor.position.x = __msg.range*std::cos(__msg.azimuth);
	pose_in_sensor.position.y = __msg.range*std::sin(__msg.azimuth);
	pose_in_sensor.position.z = 0.0;
	pose_in_sensor.orientation.x = 0.0;
	pose_in_sensor.orientation.y = 0.0;
	pose_in_sensor.orientation.z = std::sin((M_PI + __msg.azimuth - __msg.counterpart_azimuth) /2.0);
	pose_in_sensor.orientation.w = std::cos((M_PI + __msg.azimuth - __msg.counterpart_azimuth) /2.0);

	geometry_msgs::Pose pose_in_robot;
	tf2::doTransform(pose_in_sensor, pose_in_robot, T_sensor_to_robot__[__msg.header.frame_id]);

	// convert to detections
	target_detector::Detection detection_in_sensor = detection__;
	detection_in_sensor.id = __msg.anchor_id;
	detection_in_sensor.pose.pose = pose_in_sensor;
	detections_in_sensor.detections.push_back(detection_in_sensor);

	target_detector::Detection detection_in_robot = detection_in_sensor;
	detection_in_robot.pose.pose = pose_in_robot;
	detections_in_robot.detections.push_back(detection_in_robot);

	if ( publish_sensor_frame_detections__ )
		detections_sensor_frame_publisher__.publish(detections_in_sensor);
	detections_out_publisher__.publish(detections_in_robot);

	if ( vizbose__ )
		publishMarkers(detections_in_robot);
}

bool TmkUwbPerceptor::enableCallback(target_detector::DetectorEnable::Request & __request, target_detector::DetectorEnable::Response & __response)
{
	// we update id, even if already enabled
	if ( !enabled__ && __request.enable )
		subscribeToData();

	if ( enabled__ && !__request.enable )
	{
		unsubscribeFromData();

		// since we disable, make sure to publish an empty detections message to clear interested parties
		detections__.header.stamp = ros::Time::now();
		detections__.detections.clear();
		detections_out_publisher__.publish(detections__);
	}

	enabled__ = __request.enable;
	__response.success = true;
	return true;
}

bool TmkUwbPerceptor::configureParameters()
{
	perceptor_name__ = ros::this_node::getNamespace().substr(ros::this_node::getNamespace().find_last_of('/') + 1);
	return	getParamOrFail("enabled_by_default", enabled__) &&
			getParamOrFail("publish_sensor_frame_detections", publish_sensor_frame_detections__) &&
			getParamOrFail("robot_frame", robot_frame__) &&
			getParamOrFail("vizbose", vizbose__);
}

void TmkUwbPerceptor::subscribeToData()
{
	detections_in_subscriber__ = nh__.subscribe("detections_in", 1, &TmkUwbPerceptor::detectionsInCallback, this);
}

void TmkUwbPerceptor::unsubscribeFromData()
{
	detections_in_subscriber__.shutdown();
}

void TmkUwbPerceptor::publishMarkers(const target_detector::Detections & __detections)
{
	// only publish if not emtpy, otherwise rviz generates error
	if ( __detections.detections.empty() )
		return;

	marker__.header = __detections.header;
	for ( auto & detection : __detections.detections )
	{
		marker__.id = detection.id;
		marker__.pose = detection.pose.pose;
	}

	markers_publisher__.publish(marker__);
}

bool TmkUwbPerceptor::saveSensorTransform(const std_msgs::Header & __header)
{
	if ( !T_sensor_to_robot__.contains(__header.frame_id) )
	{
		geometry_msgs::TransformStamped T_sensor_to_robot;

		try
		{
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

}
