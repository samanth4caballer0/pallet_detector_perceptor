#include "target_detector/target_detector_node.h"

namespace TargetDetector
{

TargetDetectorNode::TargetDetectorNode()
{

}

TargetDetectorNode::~TargetDetectorNode()
{

}

bool TargetDetectorNode::init()
{
	// get user params
	int type;
	std::string param_str;
	double param_double;
	int param_int;
	std::map<std::string, std::string> detector_params;
	std::vector<std::string> lidar_frames;

	if ( !nh__.getParam("type", type) )
	{
		ROS_ERROR_STREAM("Failed to get DETECTOR type");
		return false;
	}

	switch ( type )
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
				ROS_WARN("Failed to read parameter clustering_distance. Using default 0.03m.");
				param_double = 0.03;
			}
			detector_params["clustering_distance"] = std::to_string(param_double);
			if ( !nh__.getParam("reflector_distance", param_double) )
			{
				ROS_WARN("Failed to read parameter reflector_distance. Using default 1.05m.");
				param_double = 1.05;
			}
			detector_params["reflector_distance"] = std::to_string(param_double);
			if ( !nh__.getParam("reflector_distance_tolerance", param_double) )
			{
				ROS_WARN("Failed to read parameter reflector_distance_tolerance. Using default 0.03m.");
				param_double = 0.03;
			}
			detector_params["reflector_distance_tolerance"] = std::to_string(param_double);
			detector__ = std::make_shared<MarkerReflector>();
			break;
		default:
			ROS_ERROR_STREAM("Unknown detector type " << type);
			return false;
			break;
	}

	// init detetector object
	if ( ! detector__->init(detector_params) )
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

	// get sensor frame
	sensor_frame__ = __goal->sensor_frame;

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
	lidar_reflector_subscriber__ = nh__.subscribe(lidar_topic, 1, &TargetDetectorNode::lidarReflectorCallback, this);

	// ACTION LOOP, while goal not reached, or timeout
	ros::Rate loop_rate(10);
	while ( 1 )
	{
		// check for external cancelation
		if ( detect_as_ptr__->isPreemptRequested() )
		{
			lidar_reflector_subscriber__.shutdown();
			detect_as_ptr__->setPreempted(detect_result, "Detection cancelled");
			return;
		}

		// relax
		loop_rate.sleep();
	}

}

void TargetDetectorNode::lidarReflectorCallback(
	const sick_safetyscanners::ExtendedLaserScanMsg & __scan)
{
	// fills data to detector
	//std::cout << "Lidar Callback" << std::endl;
	double angle;
	Eigen::Vector2d p_sensor, p_platform; //point wrt sensor, p wrt platform
	detector__->resetData();
	for (unsigned int ii=0; ii<__scan.reflektor_status.size(); ii++)
	{
		if ( __scan.reflektor_status[ii] )
		{
			angle = __scan.laser_scan.angle_min + ii*__scan.laser_scan.angle_increment;
			p_sensor.x() = __scan.laser_scan.ranges[ii]*cos(angle);
			p_sensor.y() = __scan.laser_scan.ranges[ii]*sin(angle);
			p_platform = T_platform2sensor__*p_sensor;
			detector__->addPointData(p_platform.x(),p_platform.y(),1.0);
		}
	}

	// detect
	std::vector<Eigen::Vector3d> key_points;
	std::vector<Eigen::Vector3d> positions;
	std::vector<Eigen::Quaterniond> orientations;
	std::vector<double> confidences;
	detector__->detect(key_points, positions, orientations, confidences);

	// publish markers
	publishMarkers(key_points, positions, orientations, "target_detector");

	// publish detection frames
	target_detector::Detections msg;
	msg.header.stamp = __scan.laser_scan.header.stamp;
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
	viz_markers.markers[0].color.g = 0.0;
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
		viz_markers.markers[ii+1].id = 0;
		viz_markers.markers[ii+1].type = visualization_msgs::Marker::ARROW;
		viz_markers.markers[ii+1].pose.position.x = __positions[ii].x();
		viz_markers.markers[ii+1].pose.position.y = __positions[ii].y();
		viz_markers.markers[ii+1].pose.position.z = __positions[ii].z();
		viz_markers.markers[ii+1].pose.orientation.x = __orientations[ii].x();
		viz_markers.markers[ii+1].pose.orientation.y = __orientations[ii].y();
		viz_markers.markers[ii+1].pose.orientation.z = __orientations[ii].z();
		viz_markers.markers[ii+1].pose.orientation.w = __orientations[ii].w();
		viz_markers.markers[ii+1].scale.x = 0.1;
		viz_markers.markers[ii+1].scale.y = 0.02;
		viz_markers.markers[ii+1].scale.z = 0.02;
		viz_markers.markers[ii+1].color.r = 1.0;
		viz_markers.markers[ii+1].color.g = 0.0;
		viz_markers.markers[ii+1].color.b = 0.0;
		viz_markers.markers[ii+1].color.a = 1.0;
	}

	viz_marker_publisher__.publish(viz_markers);
}

} //namespace
