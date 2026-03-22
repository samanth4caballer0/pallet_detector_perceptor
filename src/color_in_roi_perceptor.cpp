#include "color_in_roi_perceptor.h"

namespace TargetDetector
{

bool ColorInRoiPerceptor::init()
{
	if ( !configureParameters() )
	{
		ROS_ERROR("Failed to configure color in ROI perpceptor");
		return false;
	}

	tf_listener__.reset(new tf2_ros::TransformListener(tf_buffer__));
	enable_server__ = nh__.advertiseService("enable", &ColorInRoiPerceptor::enableServiceCallback, this);
	detections_publisher__ = nh__.advertise<target_detector::Detections>("detections", 1, false);

	if ( enabled__ )
		subscribeToData();

	if ( vizbose__ )
	{
		point_cloud_publisher__ = nh__.advertise<sensor_msgs::PointCloud2>("cloud_out", 1, false);
		viz_markers_publisher__ = nh__.advertise<visualization_msgs::Marker>("visuals", 1, false);
	}

	return true;
}

bool ColorInRoiPerceptor::configureParameters()
{
	perceptor_name__ = ros::this_node::getNamespace().substr(ros::this_node::getNamespace().find_last_of('/') + 1);

	std::vector<double> crop_min;
	std::vector<double> crop_max;
	std::vector<double> voxel_size;

	if ( !getParamOrFail("enabled_by_default", enabled__) ||
		!getParamOrFail("vizbose", vizbose__) ||
		!getParamOrFail("robot_frame", robot_frame__) ||
		!getParamOrFail("source_name", source_name__) ||
		!getParamOrFail("crop_min", crop_min) ||
		!getParamOrFail("crop_max", crop_max) ||
		!getParamOrFail("min_cloud_points", min_cloud_points__) ||
		!getParamOrFail("min_color_inliers", min_color_inliers__) )
	{
		return false;
	}

	if ( min_cloud_points__ < 1 )
	{
		ROS_ERROR_STREAM("Invalid parameter min_cloud_points: " << min_cloud_points__ << ". It must be >= 1.");
		return false;
	}

	if ( min_color_inliers__ < 1 )
	{
		ROS_ERROR_STREAM("Invalid parameter min_color_inliers: " << min_color_inliers__ << ". It must be >= 1.");
		return false;
	}

	if ( crop_min.size() != 3 || crop_max.size() != 3 )
	{
		ROS_ERROR("Parameters crop_min and crop_max must have exactly 3 elements.");
		return false;
	}

	crop_min__ = Eigen::Vector4f(crop_min[0], crop_min[1], crop_min[2], 1.0f);
	crop_max__ = Eigen::Vector4f(crop_max[0], crop_max[1], crop_max[2], 1.0f);

	if ( (crop_min__.head<3>().array() >= crop_max__.head<3>().array()).any() )
	{
		ROS_ERROR("Each crop_min component must be strictly lower than crop_max.");
		return false;
	}

	return true;
}

bool ColorInRoiPerceptor::enableServiceCallback(
	target_detector::DetectorEnable::Request & __request,
	target_detector::DetectorEnable::Response & __response)
{
	/*if ( __request.enable && !validateDiameter(__request.diameter) )
	{
		ROS_ERROR_STREAM("Invalid diameter for vertical cylinder detection: " << __request.diameter);
		__response.success = false;
		return true;
	}*/

	if ( __request.enable )
	{
		target_color_r__ = __request.target_red;
		target_color_g__ = __request.target_green;
		target_color_b__ = __request.target_blue;
		if ( !enabled__ )
			subscribeToData();
	}
	else // request to disable
	{
		if ( enabled__ )
		{
			unsubscribeFromData();
			target_detector::Detections msg;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = robot_frame__;
			msg.source_name = source_name__;
			detections_publisher__.publish(msg);
		}
	}

	enabled__ = __request.enable;
	__response.success = true;
	return true;
}

void ColorInRoiPerceptor::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& __cloud_in)
{
	if ( !enabled__ )
		return;

	if ( !__cloud_in )
	{
		ROS_WARN("ColorInRoiPerceptor::pointCloudCallback(): void input cloud");
		return;
	}

	if ( static_cast<int>(__cloud_in->width * __cloud_in->height) < min_cloud_points__ )
	{
		ROS_WARN("ColorInRoiPerceptor::pointCloudCallback(): too few points in the input cloud");
		return;
	}

	if ( !saveSensorTransform(__cloud_in->header) )
	{
		ROS_WARN("ColorInRoiPerceptor::pointCloudCallback(): static transform from robot to camera not available");
		return;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_crop(new pcl::PointCloud<pcl::PointXYZRGB>());
	Eigen::Isometry3d T_O_S = Eigen::Isometry3d::Identity(); // object wrt sensor
	Eigen::Isometry3d T_O_R = Eigen::Isometry3d::Identity(); // object wrt robot
	double confidence_level = 0.0;
}

void ColorInRoiPerceptor::subscribeToData()
{
	point_cloud_subscriber__ = nh__.subscribe("point_cloud_in", 1, &ColorInRoiPerceptor::pointCloudCallback, this);
}

void ColorInRoiPerceptor::unsubscribeFromData()
{
	point_cloud_subscriber__.shutdown();
}

void ColorInRoiPerceptor::publishMarkers(const target_detector::Detections & __detections_msg)
{
	if ( __detections_msg.detections.empty() )
		return;

	visualization_msgs::Marker marker_msg;
	marker_msg.header = __detections_msg.header;
	marker_msg.ns = perceptor_name__;
	marker_msg.id = 0;
	marker_msg.type = visualization_msgs::Marker::CYLINDER;
	marker_msg.action = visualization_msgs::Marker::ADD;
	marker_msg.pose.position.x = __detections_msg.detections[0].pose.pose.position.x;
	marker_msg.pose.position.y = __detections_msg.detections[0].pose.pose.position.y + 0.5;
	marker_msg.pose.position.z = __detections_msg.detections[0].pose.pose.position.z;
	marker_msg.pose.orientation.x = std::sin(M_PI / 4.0);
	marker_msg.pose.orientation.y = 0.0;
	marker_msg.pose.orientation.z = 0.0;
	marker_msg.pose.orientation.w = std::cos(M_PI / 4.0);
	marker_msg.scale.x = 1.0;
	marker_msg.scale.y = 1.0;
	marker_msg.scale.z = 1.0;
	marker_msg.color.a = 0.6;
	marker_msg.color.r = 1.0;
	marker_msg.color.g = 1.0;
	marker_msg.color.b = 0.0;
	viz_markers_publisher__.publish(marker_msg);
}

bool ColorInRoiPerceptor::saveSensorTransform(const std_msgs::Header & __header)
{
	if ( T_robot_to_sensor__.contains(__header.frame_id) )
		return true;

	geometry_msgs::TransformStamped tr_st;
	try
	{
		tr_st = tf_buffer__.lookupTransform(robot_frame__, __header.frame_id, ros::Time(0), ros::Duration(1.0));
	}
	catch (tf2::TransformException & ex)
	{
		ROS_WARN("%s", ex.what());
		ROS_WARN_STREAM("Error getting transform from " << robot_frame__ << " to " << __header.frame_id);
		return false;
	}

	T_robot_to_sensor__[__header.frame_id].translation() <<
		tr_st.transform.translation.x,
		tr_st.transform.translation.y,
		tr_st.transform.translation.z;

	Eigen::Quaterniond qq(
		tr_st.transform.rotation.w,
		tr_st.transform.rotation.x,
		tr_st.transform.rotation.y,
		tr_st.transform.rotation.z);
	T_robot_to_sensor__[__header.frame_id].linear() = qq.toRotationMatrix();
	T_robot_to_sensor__[__header.frame_id].matrix().block<1,4>(3,0) << 0, 0, 0, 1;

	ROS_INFO_STREAM("Static transform updated. From " << robot_frame__ << " to " << __header.frame_id);
	return true;
}

} // namespace TargetDetector
