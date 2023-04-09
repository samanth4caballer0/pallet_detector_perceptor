#include "target_detector/target_detector_node.h"

namespace TargetDetector
{

TargetDetectorNode::TargetDetectorNode() :
	detector_state__(target_detector::DetectFeedback::STATE_DISABLED)
{

}

TargetDetectorNode::~TargetDetectorNode()
{

}

bool TargetDetectorNode::init()
{
	// get user params
	std::string param_str;
	double param_double;
	int param_int;
	std::map<std::string, std::string> detector_params;
	std::vector<std::string> lidar_frames;

	if ( !nh__.getParam("type", detector_type__) )
	{
		ROS_ERROR("Failed to get detector type");
		return false;
	}

	if ( !nh__.getParam("topic_type", topic_type__) )
	{
		ROS_WARN("Failed to get topic_type. Using default sensor_msgs/LaserScan");
		topic_type__ = SENSOR_MSG_LASER_SCAN;
	}
	if ( (topic_type__ != SENSOR_MSG_LASER_SCAN) && ( topic_type__ != SICK_EXTENDED_LASER_SCAN) )
	{
		ROS_WARN("Invalid value for topic_type. Using default sensor_msgs/LaserScan");
		topic_type__ = SENSOR_MSG_LASER_SCAN;
	}
	if ( !nh__.getParam("reflector_intensity_threshold", reflector_intensity_threshold__) )
	{
		if ( topic_type__ == SENSOR_MSG_LASER_SCAN )
		{
			ROS_WARN("Failed to get reflector_intensity_threshold. Using default 254");
			reflector_intensity_threshold__ = 254;
		}
	}

	switch ( detector_type__ )
	{
		case DETECTOR_MARKER_REFLECTOR:
			if ( !nh__.getParam("lidar_frames", lidar_frames) )
			{
				ROS_ERROR("Failed to read parameter lidar_frames");
				return false;
			}
			for ( int i = 0; i < lidar_frames.size(); i++ )
				lidar_frame_to_topic_map__[lidar_frames.at(i)] = "laser_input"+std::to_string(i);
			if ( !nh__.getParam("clustering_distance", param_double) )
			{
				ROS_WARN("Failed to read parameter clustering_distance. Using default 0.1m.");
				param_double = 0.1;
			}
			detector_params["clustering_distance"] = std::to_string(param_double);
			if ( !nh__.getParam("reflector_distance_tolerance", param_double) )
			{
				ROS_WARN("Failed to read parameter reflector_distance_tolerance. Using default 0.03m.");
				param_double = 0.03;
			}
			detector_params["reflector_distance_tolerance"] = std::to_string(param_double);
			detector__ = std::make_shared<MarkerReflector>();
			break;
		default:
			ROS_ERROR_STREAM("Unknown detector type " << detector_type__);
			return false;
			break;
	}

	// init detetector object
	if ( !detector__->init(detector_params) )
	{
		return false;
	}

	// Action server
	detect_as_ptr__.reset(
		new actionlib::SimpleActionServer<target_detector::DetectAction>(
			nh__,
			"detect",
			boost::bind(&TargetDetectorNode::detectCallback, this, _1),
			false));
	detect_as_ptr__->start();

	// Publishers
	detector_publisher__ = nh__.advertise<target_detector::Detections>( "target_detections", 1, true );
	viz_marker_publisher__ = nh__.advertise<visualization_msgs::MarkerArray>( "target_detector_markers", 1, true );

	return true;
}

void TargetDetectorNode::detectCallback(
	const target_detector::DetectGoalConstPtr & __goal)
{
	target_detector::DetectFeedback detect_feedback;
	target_detector::DetectResult detect_result;

	// get params
	dynamic_params__.clear();
	switch ( detector_type__ )
	{
		case DETECTOR_MARKER_REFLECTOR:
			sensor_frame__ = __goal->sensor_frame;
			dynamic_params__["reflector_baseline"] = __goal->reflector_baseline;
			break;
		default:
			ROS_WARN_STREAM("Unknown detector type " << detector_type__);
			detect_as_ptr__->setAborted(detect_result, "Unknown detector type");
			return; break;
	}

	// check if sensor frame was registered at lidar_frame_to_topic_map__
	if ( lidar_frame_to_topic_map__.find(sensor_frame__) == lidar_frame_to_topic_map__.end() )
	{
		ROS_WARN("Unknown lidar frame");
		detect_as_ptr__->setAborted(detect_result, "Unknown lidar frame");
		return;
	}

	// get transform from platform to sensor frame
	ros::Time now = ros::Time::now();
	tf::StampedTransform Tstmp;
	if ( tfl__.waitForTransform("platform", sensor_frame__, now, ros::Duration(1.)) )
	{
		//look up for transform at tf tree and set the homogeneous matrix
		tfl__.lookupTransform("platform", sensor_frame__, now, Tstmp);
		T_platform2sensor__.matrix() <<
			Tstmp.getBasis().getRow(0).x(), Tstmp.getBasis().getRow(0).y(), Tstmp.getOrigin().x(),
			Tstmp.getBasis().getRow(1).x(), Tstmp.getBasis().getRow(1).y(), Tstmp.getOrigin().y(),
			0, 0, 1.0;
	}
	else //some error occurred
	{
		ROS_WARN_STREAM("Required Transform from platform to "<< sensor_frame__ << " not found");
		detect_as_ptr__->setAborted(detect_result, "Unknown platform to sensor transform");
		return;
	}

	// subscribe to requested lidar
	std::string lidar_topic = lidar_frame_to_topic_map__[sensor_frame__];
	switch (topic_type__)
	{
		case SENSOR_MSG_LASER_SCAN:
			lidar_reflector_subscriber__ =
				nh__.subscribe(lidar_topic, 1, &TargetDetectorNode::laserScanCallback, this);
			break;
		case SICK_EXTENDED_LASER_SCAN:
			lidar_reflector_subscriber__ =
				nh__.subscribe(lidar_topic, 1, &TargetDetectorNode::laserScanExtendedCallback, this);
			break;
		default:
			ROS_WARN("Unknown topic_type");
			detect_as_ptr__->setAborted(detect_result, "Unknown topic_type");
			return;
			break;
	}

	// ACTION LOOP, while goal not reached, or timeout
	ros::Rate loop_rate(10);
	detector_state__ = target_detector::DetectFeedback::STATE_ENABLED;
	detecting_flag__ = false;
	while ( ros::ok() )
	{
		// check for external cancelation
		if ( detect_as_ptr__->isPreemptRequested() )
		{
			lidar_reflector_subscriber__.shutdown();
			detector_state__ = target_detector::DetectFeedback::STATE_DISABLED;
			detect_as_ptr__->setPreempted(detect_result, "Detection cancelled");
			return;
		}

		// publish feedback
		detect_feedback.detector_state = detector_state__;
		detect_feedback.detecting = detecting_flag__;
		detect_as_ptr__->publishFeedback(detect_feedback);

		// relax
		loop_rate.sleep();
	}
}

void TargetDetectorNode::laserScanCallback(
	const sensor_msgs::LaserScan & __scan)
{
	// adds data to detector
	double calib_delta, range, angle;
	Eigen::Vector2d p_sensor, p_platform; //point wrt sensor, p wrt platform
	detector__->resetData();
	for (unsigned int ii=0; ii<__scan.intensities.size(); ii++)
	{
		if ( __scan.intensities[ii] > reflector_intensity_threshold__)
		{
			// apply simple calibration model for range in reflector points
			if (__scan.ranges[ii] < 2.0 )
				calib_delta = -0.01*__scan.ranges[ii]+0.02;
			else
				calib_delta = 0;
			range =  __scan.ranges[ii] + calib_delta;
			angle = __scan.angle_min + ii*__scan.angle_increment;
			p_sensor.x() = range*cos(angle);
			p_sensor.y() = range*sin(angle);
			p_platform = T_platform2sensor__*p_sensor;
			detector__->addPointData(p_platform.x(),p_platform.y(),1.0);
		}
	}

	// detect
	detect(__scan.header.stamp);
}

void TargetDetectorNode::laserScanExtendedCallback(
	const sick_safetyscanners::ExtendedLaserScanMsg & __scan)
{
	// adds data to detector
	double calib_delta, range, angle;
	Eigen::Vector2d p_sensor, p_platform; //point wrt sensor, p wrt platform
	detector__->resetData();
	for (unsigned int ii=0; ii<__scan.reflektor_status.size(); ii++)
	{
		if ( __scan.reflektor_status[ii] )
		{
			// apply simple calibration model for range in reflector points
			if (__scan.laser_scan.ranges[ii] < 2.0 )
				calib_delta = -0.01*__scan.laser_scan.ranges[ii]+0.02;
			else
				calib_delta = 0;
			range =  __scan.laser_scan.ranges[ii] + calib_delta;
			angle = __scan.laser_scan.angle_min + ii*__scan.laser_scan.angle_increment;
			p_sensor.x() = range*cos(angle);
			p_sensor.y() = range*sin(angle);
			p_platform = T_platform2sensor__*p_sensor;
			detector__->addPointData(p_platform.x(),p_platform.y(),1.0);
		}
	}

	// detect
	detect(__scan.laser_scan.header.stamp);
}

void TargetDetectorNode::detect(const ros::Time & __stamp)
{
	// detect, assuming data has been already added to detector
	std::vector<Eigen::Vector3d> key_points;
	std::vector<Eigen::Vector3d> positions;
	std::vector<Eigen::Quaterniond> orientations;
	std::vector<double> confidences;
	detector__->detect(dynamic_params__, key_points, positions, orientations, confidences);
	if ( positions.empty() )
		detecting_flag__= false;
	else
		detecting_flag__= true;

	// publish markers
	publishMarkers(key_points, positions, orientations, "target_detector");

	// publish detection frames
	target_detector::Detections msg;
	msg.header.stamp = __stamp;
	msg.header.frame_id = "platform";
	msg.poses.resize(positions.size());
	for (unsigned int ii=0; ii<positions.size(); ii++)
	{
		msg.poses[ii].position.x = positions[ii].x();
		msg.poses[ii].position.y = positions[ii].y();
		msg.poses[ii].position.z = positions[ii].z();
		msg.poses[ii].orientation.x = orientations[ii].x();
		msg.poses[ii].orientation.y = orientations[ii].y();
		msg.poses[ii].orientation.z = orientations[ii].z();
		msg.poses[ii].orientation.w = orientations[ii].w();
	}
	detector_publisher__.publish(msg);
}

void TargetDetectorNode::publishMarkers(
	const std::vector<Eigen::Vector3d> & __key_points,
	const std::vector<Eigen::Vector3d> & __positions,
	const std::vector<Eigen::Quaterniond> & __orientations,
	const std::string __marker_namespace) const
{
	visualization_msgs::MarkerArray viz_markers;

	ros::Time now = ros::Time::now();
	viz_markers.markers.resize(__positions.size()+1);

	// spheres for __key_points
	viz_markers.markers[0].header.stamp = now;
	viz_markers.markers[0].header.frame_id = "platform";
	viz_markers.markers[0].ns = __marker_namespace;
	viz_markers.markers[0].action = visualization_msgs::Marker::ADD;
	viz_markers.markers[0].lifetime = ros::Duration(1.0);
	viz_markers.markers[0].id = 0;
	viz_markers.markers[0].type = visualization_msgs::Marker::SPHERE_LIST;
	viz_markers.markers[0].pose.position.x = 0;
	viz_markers.markers[0].pose.position.y = 0;
	viz_markers.markers[0].pose.position.z = 0;
	viz_markers.markers[0].pose.orientation.x = 0.0;
	viz_markers.markers[0].pose.orientation.y = 0.0;
	viz_markers.markers[0].pose.orientation.z = 0.0;
	viz_markers.markers[0].pose.orientation.w = 1.0;
	viz_markers.markers[0].scale.x = 0.04;
	viz_markers.markers[0].scale.y = 0.04;
	viz_markers.markers[0].scale.z = 0.04;
	viz_markers.markers[0].color.r = 1.0;
	viz_markers.markers[0].color.g = 1.0;
	viz_markers.markers[0].color.b = 0.0;
	viz_markers.markers[0].color.a = 1.0;
	viz_markers.markers[0].points.resize(__key_points.size());
	for ( unsigned int ii=0; ii<__key_points.size(); ii++ )
	{
		viz_markers.markers[0].points[ii].x = __key_points[ii].x();
		viz_markers.markers[0].points[ii].y = __key_points[ii].y();
		viz_markers.markers[0].points[ii].z = 0.0;
	}

	// arrows for positions/orientations
	for ( unsigned int ii=0; ii<__positions.size(); ii++ )
	{
		viz_markers.markers[ii+1].header.stamp = now;
		viz_markers.markers[ii+1].header.frame_id = "platform";
		viz_markers.markers[ii+1].ns = __marker_namespace;
		viz_markers.markers[ii+1].action = visualization_msgs::Marker::ADD;
		viz_markers.markers[ii+1].lifetime = ros::Duration(1.0);
		viz_markers.markers[ii+1].id = ii+1;
		viz_markers.markers[ii+1].type = visualization_msgs::Marker::ARROW;
		viz_markers.markers[ii+1].pose.position.x = __positions[ii].x();
		viz_markers.markers[ii+1].pose.position.y = __positions[ii].y();
		viz_markers.markers[ii+1].pose.position.z = __positions[ii].z();
		viz_markers.markers[ii+1].pose.orientation.x = __orientations[ii].x();
		viz_markers.markers[ii+1].pose.orientation.y = __orientations[ii].y();
		viz_markers.markers[ii+1].pose.orientation.z = __orientations[ii].z();
		viz_markers.markers[ii+1].pose.orientation.w = __orientations[ii].w();
		viz_markers.markers[ii+1].scale.x = 0.2;
		viz_markers.markers[ii+1].scale.y = 0.02;
		viz_markers.markers[ii+1].scale.z = 0.02;
		viz_markers.markers[ii+1].color.r = 1.0;
		viz_markers.markers[ii+1].color.g = 1.0;
		viz_markers.markers[ii+1].color.b = 0.0;
		viz_markers.markers[ii+1].color.a = 1.0;
	}

	viz_marker_publisher__.publish(viz_markers);
}

} //namespace
