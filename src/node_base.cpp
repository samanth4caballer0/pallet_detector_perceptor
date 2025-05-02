#include "target_detector/node_base.h"

namespace TargetDetector
{

NodeBase::NodeBase() : nh__("~")
{
	//
}

NodeBase::~NodeBase()
{
	//
}

bool NodeBase::init()
{
	// get params
	int int_param;
	if ( !nh__.getParam("detector_type", int_param) )
	{
		ROS_ERROR_STREAM("Failed to get detector_type");
		return false;
	}
	else
	{
		detector_type__ = (uint8_t)int_param;
	}
	if ( !nh__.getParam("robot_frame", robot_frame__) )
	{
		ROS_ERROR("Failed to get robot_frame parameter");
		return false;
	}
	if ( !nh__.getParam("vizbose", vizbose__) )
	{
		ROS_ERROR("Failed to get vizbose parameter");
		return false;
	}

	// publishers
	detector_publisher__ = nh__.advertise<target_detector::Detections>( "detections", 1, true );

	// tf 2
	tf_listener__.reset(new tf2_ros::TransformListener(tf_buffer__));

	return true;
}

bool NodeBase::saveSensorTransform(const std_msgs::Header & __header)
{
 	geometry_msgs::TransformStamped Trs; // from robot to sensor
	Eigen::Quaterniond aux_qt;
	double angle_z;

	// if frame_id not already in the transforms map, keep it
	if ( T_robot_to_sensor__.find(__header.frame_id) == T_robot_to_sensor__.end() )
	{
		try
		{
			// get transform from tf
			Trs = tf_buffer__.lookupTransform(__header.frame_id, robot_frame__, __header.stamp, ros::Duration(1.0));

			// convert to Eigen Isometry3d
			aux_qt.coeffs() <<
				Trs.transform.rotation.x,
				Trs.transform.rotation.y,
				Trs.transform.rotation.z,
				Trs.transform.rotation.w;
			T_robot_to_sensor__[__header.frame_id].linear() = aux_qt.matrix();
			T_robot_to_sensor__[__header.frame_id].translation() <<
				Trs.transform.translation.x,
				Trs.transform.translation.y,
				Trs.transform.translation.z;

			// convert to Eigen::Isometry2d
			angle_z = 2*std::atan2(Trs.transform.rotation.z, Trs.transform.rotation.w);
			T_robot_to_sensor_2d__[__header.frame_id].matrix() <<
				std::cos(angle_z), -std::sin(angle_z), Trs.transform.translation.x,
				std::sin(angle_z),  std::cos(angle_z), Trs.transform.translation.y,
				0,0,1;

			//std::cout << "Saved Transform from " << robot_frame__ << "to " << __header.frame_id << std::endl;
			//std::cout << T_robot_to_sensor_2d__[__header.frame_id].matrix() << std::endl;
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

} // end of namespace
