#include "target_detector/node_lidar_2d.h"

namespace TargetDetector
{

NodeLidar2d::NodeLidar2d()
{
	//
}

NodeLidar2d::~NodeLidar2d()
{
	//
}

bool NodeLidar2d::init()
{
	// base init with common configs
	if ( !NodeBase::init() )
	{
		return false;
	}

	// get params for lidar detectors
	std::vector<std::string> lidars;
	if ( !nh__.getParam("lidars", lidars) )
	{
		ROS_ERROR("Failed to get lidars parameter");
		return false;
	}
	if ( !nh__.getParam("target_size", detector_params__["target_size"]) )
	{
		ROS_ERROR("Failed to get target_size.");
		return false;
	}
	if ( !nh__.getParam("max_target_range", detector_params__["max_target_range"]) )
	{
		ROS_ERROR("Failed to get max_target_range.");
		return false;
	}

	// create the detector according detector_type__
	switch ( detector_type__ )
	{
		case target_detector::Detection::TYPE_REFLECTOR_FROM_INTENSITY:
			ROS_INFO("Creating a DetectorReflectorFromIntensity.");
			if ( !nh__.getParam("min_reflector_intensity", detector_params__["min_reflector_intensity"]) )
			{
				ROS_ERROR("Failed to get min_reflector_intensity");
				return false;
			}
			for ( auto & lidar : lidars )
			{
				lidar_subscribers__.push_back(nh__.subscribe(lidar, 1, &NodeLidar2d::laserScanCallback, this));
			}
			detector__ = std::make_shared<DetectorReflector>();
			break;
		case target_detector::Detection::TYPE_REFLECTOR_FROM_BOOL:
			ROS_INFO("Creating a DetectorReflectorFromBool.");
			for ( auto & lidar : lidars )
			{
				lidar_subscribers__.push_back(nh__.subscribe(lidar, 1, &NodeLidar2d::extendedLaserScanCallback, this));
			}
			detector__ = std::make_shared<DetectorReflector>();
			break;
		case target_detector::Detection::TYPE_REFLECTOR_PAIR:
			ROS_ERROR("Detector type not yet implemented. Exit.");
			return false;
			break;
		case target_detector::Detection::TYPE_COLUMN:
			ROS_INFO("Creating a DetectorColumn.");
			for ( auto & lidar : lidars )
			{
				lidar_subscribers__.push_back(nh__.subscribe(lidar, 1, &NodeLidar2d::laserScanCallback, this));
			}
			detector__ = std::make_shared<DetectorColumn>();
			break;
		case target_detector::Detection::TYPE_STRAIGHT_SEGMENT:
			ROS_ERROR("Detector type not yet implemented. Exit.");
			return false;
			break;
		case target_detector::Detection::TYPE_CORNER:
			ROS_ERROR("Detector type not yet implemented. Exit.");
			return false;
			break;
		case target_detector::Detection::TYPE_UNKNOWN:
		default:
			ROS_ERROR("Unknown detector type. Exit.");
			return false;
			break;
	}

	// visuals
	if ( vizbose__ )
	{
		viz_marker_publisher__ = nh__.advertise<visualization_msgs::Marker>( "visuals", 1, true );
	}

	// configure detector
	detector__->configure(detector_params__);

	// reconfigure service
	reconfigure_callback__ = boost::bind(&NodeLidar2d::reconfigureCallback, this, _1, _2);
	reconfigure_server__.setCallback(reconfigure_callback__);

	return true;
}

void NodeLidar2d::laserScanCallback(const sensor_msgs::LaserScanConstPtr & __scan_ptr)
{
	// check if platform->lidar transform exists, and save it if not
	if ( !saveSensorTransform(__scan_ptr->header) )
	{
		ROS_WARN("NodeLidar2d::laserScanCallback(): scan not processed because missing tf");
		return;
	}
	detections__.clear();
	reconfigure_mutex__.lock();
	detector__->detect(
		__scan_ptr->angle_min,
		__scan_ptr->angle_max,
		__scan_ptr->ranges,
		__scan_ptr->intensities,
		T_robot_to_sensor_2d__[__scan_ptr->header.frame_id],
		detections__);
	reconfigure_mutex__.unlock();
	publishDetections(__scan_ptr->header);
	if ( vizbose__ ) publishMarkers(__scan_ptr->header);
}

void NodeLidar2d::extendedLaserScanCallback(const sick_safetyscanners::ExtendedLaserScanMsgConstPtr & __extended_scan_ptr)
{
/*	std::vector<bool> reflector_hits;
	for ( auto & reflector_status : __extended_scan_ptr->reflektor_status )
		reflector_hits.push_back( reflector_status ? true : false);
	findReflectors(reflector_hits, __extended_scan_ptr->laser_scan);*/
}

void NodeLidar2d::reconfigureCallback(target_detector::target_detectorConfig & __config, uint32_t __level)
{
	// we want to keep the params in the yaml, so avoid taking defaults from dynamic reconfigure
	if ( first_dynamic_reconfigure__ )
	{
		first_dynamic_reconfigure__ = false;
		return;
	}

	ROS_INFO("Dynamic Reconfigure Request to TargetDetector::Node");
	detector_params__["min_reflector_intensity"] = __config.min_reflector_intensity;
	detector_params__["target_size"] = __config.target_size;
	reconfigure_mutex__.lock();
	detector__->configure(detector_params__);
	reconfigure_mutex__.unlock();
}

void NodeLidar2d::publishDetections(const std_msgs::Header & __header)
{
	target_detector::Detections msg;

	// keep the time stamp from the original header
	msg.header = __header;
	msg.header.frame_id = robot_frame__;

	// fill the message according the detector type
	switch( detector_type__ )
	{
		case target_detector::Detection::TYPE_REFLECTOR_FROM_INTENSITY:
		case target_detector::Detection::TYPE_REFLECTOR_FROM_BOOL:
		case target_detector::Detection::TYPE_COLUMN:
			msg.detections.resize( detections__.size() / 6 ); // each detection are 6 doubles
			for (unsigned int ii=0; ii<msg.detections.size(); ii++)
			{
				//[size, intensity, x0,y0,cxx0,cyy0, ... ]
				msg.detections[ii].type = detector_type__;
				msg.detections[ii].id = -1; // no id
				msg.detections[ii].pose.pose.position.x = detections__[ii*6+2];
				msg.detections[ii].pose.pose.position.y = detections__[ii*6+3];
				msg.detections[ii].pose.pose.position.z = 0.;
				msg.detections[ii].pose.pose.orientation.x = 0.;
				msg.detections[ii].pose.pose.orientation.y = 0.;
				msg.detections[ii].pose.pose.orientation.z = 0.;
				msg.detections[ii].pose.pose.orientation.w = 1.;
				msg.detections[ii].pose.covariance[0] = detections__[ii*6+4];
				msg.detections[ii].pose.covariance[7] = detections__[ii*6+5];
				msg.detections[ii].intensity = detections__[ii*6+1];
				msg.detections[ii].supports = detections__[ii*6];
				msg.detections[ii].baseline = 0.;
			}
			break;

		default:
			break;
	}

	// publish message
	detector_publisher__.publish(msg);
}

void NodeLidar2d::publishMarkers(const std_msgs::Header & __header)
{
	visualization_msgs::Marker msg;

	// fill common parts of the message
	msg.header = __header; // keep the time stamp from the original header
	msg.header.frame_id = robot_frame__;
	msg.id = 0;
	msg.action = visualization_msgs::Marker::ADD;
	msg.lifetime = ros::Duration(0.5);

	// fill default values of the message
	msg.pose.orientation.x = 0.;
	msg.pose.orientation.y = 0.;
	msg.pose.orientation.z = 0.;
	msg.pose.orientation.w = 1.;
	msg.scale.x = 0.3;
	msg.scale.y = 0.3;
	msg.scale.z = 0.3;

	// fill the message according the detector type
	switch( detector_type__ )
	{
		case target_detector::Detection::TYPE_REFLECTOR_FROM_INTENSITY:
		case target_detector::Detection::TYPE_REFLECTOR_FROM_BOOL:
			msg.ns = "reflectors";
			msg.type = visualization_msgs::Marker::SPHERE_LIST;
			msg.color.r = 1.0;
			msg.color.g = 1.0;
			msg.color.b = 0.0;
			msg.color.a = 0.75;
			msg.points.resize( detections__.size() / 6 ); // each detection are 6 doubles
			for (unsigned int ii=0; ii<msg.points.size(); ii++)
			{
				//position according [size, intensity, x0,y0,cxx0,cyy0, ... ]
				msg.points[ii].x = detections__[ii*6+2];
				msg.points[ii].y = detections__[ii*6+3];
				msg.points[ii].z = 0.;
			}
			break;

		case target_detector::Detection::TYPE_COLUMN:
			msg.ns = "columns";
			msg.type = visualization_msgs::Marker::SPHERE_LIST;
			msg.color.r = 0.0;
			msg.color.g = 1.0;
			msg.color.b = 1.0;
			msg.color.a = 0.75;
			msg.points.resize( detections__.size() / 6 ); // each detection are 6 doubles
			for (unsigned int ii=0; ii<msg.points.size(); ii++)
			{
				//position according [size, intensity, x0,y0,cxx0,cyy0, ... ]
				msg.points[ii].x = detections__[ii*6+2];
				msg.points[ii].y = detections__[ii*6+3];
				msg.points[ii].z = 0.;
			}
			break;

		default:
			break;
	}

	// publish message
	viz_marker_publisher__.publish(msg);
}

} // end of namespace
