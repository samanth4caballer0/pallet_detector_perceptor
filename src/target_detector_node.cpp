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

	// visualization markers
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

	// get transform from platform to sensor frame
	// TO DO

	// subscribe to requested lidar
	if ( lidar_frame_to_topic_map__.find(sensor_frame__) == lidar_frame_to_topic_map__.end() )
	{
		ROS_WARN("TargetDetectorNode: Unknown lidar frame");
		detect_as_ptr__->setAborted(detect_result, "Unknown lidar frame");
		return;
	}
	std::string lidar_topic = lidar_frame_to_topic_map__[sensor_frame__];
	lidar_reflector_subscriber__ = nh__.subscribe(lidar_topic, 1, &TargetDetectorNode::lidarReflectorCallback, this);

	// ACTION LOOP, while goal not reached, or timeout
	ros::Rate loop_rate(10);
	while ( 1 )
	{
		// check for external cancelation
		if ( detect_as_ptr__->isPreemptRequested() )
		{
			detect_as_ptr__->setPreempted(detect_result, "Detection cancelled");
			lidar_reflector_subscriber__.shutdown();
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
	std::cout << "Lidar Callback" << std::endl;
	double px, py, angle;
	detector__->resetData();
	for (unsigned int ii=0; ii<__scan.reflektor_status.size(); ii++)
	{
		if ( __scan.reflektor_status[ii] )
		{
			angle = __scan.laser_scan.angle_min + ii*__scan.laser_scan.angle_increment;
			px = __scan.laser_scan.ranges[ii]*cos(angle);
			py = __scan.laser_scan.ranges[ii]*sin(angle);
			detector__->addPointData(px,py,1.0);
		}
	}

	// detect
	std::vector<Eigen::Vector3d> key_points;
	std::vector<Eigen::Vector3d> positions;
	std::vector<Eigen::Quaterniond> orientations;
	std::vector<double> confidences;
	detector__->detect(key_points, positions, orientations, confidences);
	std::cout << "key_points.size(): " << key_points.size() << std::endl;

	// publish markers
	publishMarkers(key_points, 0.5, "target_detector");

}

void TargetDetectorNode::publishMarkers(
	const std::vector<Eigen::Vector3d> & __points,
	const double & __red_color,
	const std::string __marker_namespace) const
{
	visualization_msgs::MarkerArray viz_markers;

	viz_markers.markers.resize(1);
	viz_markers.markers[0].header.stamp = ros::Time::now();
	viz_markers.markers[0].header.frame_id = sensor_frame__;
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
	viz_markers.markers[0].color.r = __red_color;
	viz_markers.markers[0].color.g = 1.0-__red_color;
	viz_markers.markers[0].color.b = 0.0;
	viz_markers.markers[0].color.a = 1.0;
	viz_markers.markers[0].points.resize(__points.size());
	for ( unsigned int ii=0; ii<__points.size(); ii++ )
	{
		viz_markers.markers[0].points[ii].x = __points[ii].x();
		viz_markers.markers[0].points[ii].y = __points[ii].y();
		viz_markers.markers[0].points[ii].z = 0.0;
	}
	viz_marker_publisher__.publish(viz_markers);
}

} //namespace
