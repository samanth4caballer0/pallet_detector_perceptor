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
			lidar_frame_to_topic_map__[lidar_frames.at(i)] = "laser_input"+std::to_string(i+1);
			if ( !nh__.getParam("clustering_distance", param_double) )
			{
				ROS_WARN("Failed to read parameter clustering_distance. Using default 0.03m.");
			}
			detector_params["clustering_distance"] = std::to_string(param_double);
			if ( !nh__.getParam("reflector_distance", param_double) )
			{
				ROS_WARN("Failed to read parameter reflector_distance. Using default 1.05m.");
			}
			detector_params["reflector_distance"] = std::to_string(param_double);
			if ( !nh__.getParam("reflector_distance_tolerance", param_double) )
			{
				ROS_WARN("Failed to read parameter reflector_distance_tolerance. Using default 1.05m.");
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

	return true;
}

void TargetDetectorNode::detectCallback(
	const target_detector::DetectGoalConstPtr & __goal)
{

}

void TargetDetectorNode::lidarCallback(
	const sensor_msgs::LaserScan & __scan)
{

}

void TargetDetectorNode::publishMarkers(
	const std::vector<Cluster> & __clusters,
	const double & __red_color,
	const std::string __marker_namespace) const
{

}

} //namespace
