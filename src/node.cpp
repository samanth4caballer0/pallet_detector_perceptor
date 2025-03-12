#include "target_detector/node.h"

namespace TargetDetector
{

Node::Node() : nh__("~")
{
	//
}

Node::~Node()
{
	//
}

bool Node::init()
{
	// get params
	std::vector<std::string> lidars;
	std::string scan_type;
	int detector_type;
	if ( !nh__.getParam("vizbose", vizbose__) )
	{
		ROS_ERROR("Failed to get vizbose parameter");
		return false;
	}
	if ( !nh__.getParam("detector_type", detector_type) )
	{
		ROS_ERROR_STREAM("Failed to get BMS detector_type");
		return false;
	}
	if ( !nh__.getParam("lidars", lidars) )
	{
		ROS_ERROR("Failed to get lidars parameter");
		return false;
	}
	if ( !nh__.getParam("robot_frame", robot_frame__) )
	{
		ROS_ERROR("Failed to get robot_frame parameter");
		return false;
	}
	if ( !nh__.getParam("scan_type", scan_type) )
	{
		ROS_ERROR("Failed to get scan_type parameter");
		return false;
	}
	if ( scan_type == "sensor_msgs/LaserScan" ) // if the laser message is the standard, the reflector intensity threshold is required
	{
		if ( !nh__.getParam("min_reflector_intensity", detector_params__["min_reflector_intensity"]) )
		{
			ROS_ERROR("Failed to get min_reflector_intensity");
			return false;
		}
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

	// create the detector according detector_type
	switch ( detector_type )
	{
		case REFLECTOR:
			ROS_INFO("Creating a DetectorReflector.");
			detector__ = std::make_shared<DetectorReflector>();
			break;
		case REFLECTOR_PAIR:
			ROS_ERROR("Detector type not yet implemented. Exit.");
			return false;
			break;
		case COLUMN:
			ROS_INFO("Creating a DetectorColumn.");
			detector__ = std::make_shared<DetectorColumn>();
			break;
		case STRAIGHT_SEGMENT:
			ROS_ERROR("Detector type not yet implemented. Exit.");
			return false;
			break;
		case CORNER:
			ROS_ERROR("Detector type not yet implemented. Exit.");
			return false;
			break;
		case PALLET:
			ROS_ERROR("Detector type not yet implemented. Exit.");
			return false;
			break;
		case ALVAR:
			ROS_ERROR("Detector type not yet implemented. Exit.");
			return false;
			break;
		case UNKNOWN:
		default:
			ROS_ERROR("Unknown detector type. Exit.");
			return false;
			break;
	}

	// configure detector
	detector__->configure(detector_params__);

	// subscribe to lidar topics, according scan_type
	for ( auto & lidar : lidars )
	{
		if ( scan_type == "sensor_msgs/LaserScan" )
			lidar_subscribers__.push_back(nh__.subscribe(lidar, 1, &Node::laserScanCallback, this));
		else if ( scan_type == "sick_safetyscanners/ExtendedLaserScanMsg" )
			lidar_subscribers__.push_back(nh__.subscribe(lidar, 1, &Node::extendedLaserScanCallback, this));
		else
			return false;
	}

	// publishers
	detetctor_publisher__ = nh__.advertise<target_detector::Detections>( "detections", 1, true );
	if ( vizbose__ ) viz_marker_publisher__ = nh__.advertise<visualization_msgs::Marker>( "viz_markers", 1, true );

	// reconfigure service
	reconfigure_callback__ = boost::bind(&Node::reconfigureCallback, this, _1, _2);
	reconfigure_server__.setCallback(reconfigure_callback__);

	// tf 2
	tf_listener__.reset(new tf2_ros::TransformListener(tf_buffer__));

	return true;
}

void Node::laserScanCallback(const sensor_msgs::LaserScanConstPtr & __scan_ptr)
{
	// check if platform->lidar transform exists, and save it if not
	if ( !saveLidarTransform(__scan_ptr->header) )
	{
		ROS_WARN("Node::laserScanCallback(): scan not processed because missing tf");
	}
	detections__.clear();
	reconfigure_mutex__.lock();
	detector__->detect(
		__scan_ptr->angle_min,
		__scan_ptr->angle_max,
		__scan_ptr->ranges,
		__scan_ptr->intensities,
		T_robot_to_lidar__[__scan_ptr->header.frame_id],
		detections__);
	reconfigure_mutex__.unlock();
	publishDetections(__scan_ptr->header);
	if ( vizbose__ ) publishMarkers(__scan_ptr->header);
}

void Node::extendedLaserScanCallback(const sick_safetyscanners::ExtendedLaserScanMsgConstPtr & __extended_scan_ptr)
{
/*	std::vector<bool> reflector_hits;
	for ( auto & reflector_status : __extended_scan_ptr->reflektor_status )
		reflector_hits.push_back( reflector_status ? true : false);
	findReflectors(reflector_hits, __extended_scan_ptr->laser_scan);*/
}

void Node::reconfigureCallback(target_detector::target_detectorConfig & __config, uint32_t __level)
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

bool Node::saveLidarTransform(const std_msgs::Header & __header)
{
 	geometry_msgs::TransformStamped Trl; // from robot to lidar
	double angle_z;
	// if frame_id not already in the transforms map, keep it
	if ( T_robot_to_lidar__.find(__header.frame_id) == T_robot_to_lidar__.end() )
	{
		try
		{
			//T_lidar_to_robot__[__header.frame_id] = tf_buffer__.lookupTransform(robot_frame__, __header.frame_id, __header.stamp, ros::Duration(1.0));
			Trl = tf_buffer__.lookupTransform(__header.frame_id, robot_frame__, __header.stamp, ros::Duration(1.0));

			// convert to Eigen::Isometry2d
			angle_z = 2*std::atan2(Trl.transform.rotation.z, Trl.transform.rotation.w);
			T_robot_to_lidar__[__header.frame_id].matrix() <<
				std::cos(angle_z), -std::sin(angle_z), Trl.transform.translation.x,
				std::sin(angle_z),  std::cos(angle_z), Trl.transform.translation.y,
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

void Node::publishDetections(const std_msgs::Header & __header)
{
	target_detector::Detections msg;

	// keep the time stamp from the original header
	msg.header = __header;
	msg.header.frame_id = robot_frame__;

	// fill the message according the detector type
	switch( detector__->type() )
	{
		case REFLECTOR:
		case COLUMN:
			msg.detections.resize( detections__.size() / 6 ); // each detection are 6 doubles
			for (unsigned int ii=0; ii<msg.detections.size(); ii++)
			{
				//[size, intensity, x0,y0,cxx0,cyy0, ... ]
				msg.detections[ii].type = detector__->type();
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
	detetctor_publisher__.publish(msg);
}

void Node::publishMarkers(const std_msgs::Header & __header)
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
	switch( detector__->type() )
	{
		case REFLECTOR:
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

		case COLUMN:
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
