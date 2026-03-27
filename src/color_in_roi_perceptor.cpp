#include "color_in_roi_perceptor.h"

#include <cmath>

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

	if ( vizbose__ )
	{
		point_cloud_publisher__ = nh__.advertise<sensor_msgs::PointCloud2>("cloud_out", 1, false);
		viz_markers_publisher__ = nh__.advertise<visualization_msgs::Marker>("visuals", 1, false);
	}

	if ( enabled__ )
		subscribeToData();

	return true;
}

bool ColorInRoiPerceptor::configureParameters()
{
	perceptor_name__ = ros::this_node::getNamespace().substr(ros::this_node::getNamespace().find_last_of('/') + 1);

	std::vector<double> crop_min;
	std::vector<double> crop_max;
	double red_hue_center_deg;
	double green_hue_center_deg;
	double blue_hue_center_deg;
	std::string default_color_name;
	if ( !getParamOrFail("enabled_by_default", enabled__) ||
		!getParamOrFail("vizbose", vizbose__) ||
		!getParamOrFail("robot_frame", robot_frame__) ||
		!getParamOrFail("source_name", source_name__) ||
		!getParamOrFail("default_color", default_color_name) ||
		!getParamOrFail("red_hue_center_deg", red_hue_center_deg) ||
		!getParamOrFail("green_hue_center_deg", green_hue_center_deg) ||
		!getParamOrFail("blue_hue_center_deg", blue_hue_center_deg) ||
		!getParamOrFail("crop_min", crop_min) ||
		!getParamOrFail("crop_max", crop_max) ||
		!getParamOrFail("min_cloud_points", min_cloud_points__) ||
		!getParamOrFail("min_color_inliers_points", min_color_inliers_points__) )
	{
		return false;
	}

	if ( min_cloud_points__ < 1 )
	{
		ROS_ERROR_STREAM("Invalid parameter min_cloud_points: " << min_cloud_points__ << ". It must be >= 1.");
		return false;
	}

	if ( min_color_inliers_points__ < 1 )
	{
		ROS_ERROR_STREAM("Invalid parameter min_color_inliers_points: " << min_color_inliers_points__ << ". It must be >= 1.");
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

	std::map<uint8_t, float> color_tag_to_hue;
	color_tag_to_hue[target_detector::Color::RED] = static_cast<float>(red_hue_center_deg);
	color_tag_to_hue[target_detector::Color::GREEN] = static_cast<float>(green_hue_center_deg);
	color_tag_to_hue[target_detector::Color::BLUE] = static_cast<float>(blue_hue_center_deg);
	if ( !detector__.configure(crop_min__, crop_max__, min_color_inliers_points__, color_tag_to_hue) )
	{
		ROS_ERROR("Failed to configure ColorInRoiDetector");
		return false;
	}

	if ( !parseColorCode(default_color_name, target_color__) )
	{
		ROS_ERROR_STREAM("Invalid parameter default_color: " << default_color_name << ". Use red, green, blue, or unknown.");
		return false;
	}
	// std::cout << "target_color: " << (unsigned int)target_color__ << std::endl;

	return true;
}

bool ColorInRoiPerceptor::enableServiceCallback(
	target_detector::DetectorEnable::Request & __request,
	target_detector::DetectorEnable::Response & __response)
{
	if ( __request.enable )
	{
		target_color__ = __request.color.code;
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
	// basic checks
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

	// save sensor transform (will be executed once)
	if ( !saveSensorTransform(__cloud_in->header) )
	{
		ROS_WARN("ColorInRoiPerceptor::pointCloudCallback(): static transform from robot to camera not available");
		return;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(*__cloud_in, *cloud_in);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZRGB>());
	Detectors::ColorInRoiDetection detection_in_sensor;
	const bool detection_found = detector__.detect(cloud_in, cloud_roi, detection_in_sensor);

	if ( vizbose__ )
		point_cloud_publisher__.publish(cloud_roi);

	if ( !detection_found )
		return;

	// UNKNOWN acts as a wildcard target so a single detector instance can report
	// whichever of red/green/blue wins the ROI classification.
	if ( target_color__ != target_detector::Color::UNKNOWN &&
		detection_in_sensor.detected_color != target_color__ )
		return;

	geometry_msgs::Pose pose_in_robot;
	tf2::doTransform(detection_in_sensor.pose_in_sensor, pose_in_robot, T_sensor_to_robot__[__cloud_in->header.frame_id]);

	target_detector::Detections detections_msg;
	detections_msg.header = __cloud_in->header;
	detections_msg.header.frame_id = robot_frame__;
	detections_msg.source_name = source_name__;
	detections_msg.detections.resize(1);
	detections_msg.detections[0].type = target_detector::Detection::COLOR_IN_ROI;
	detections_msg.detections[0].pose.pose = pose_in_robot;
	detections_msg.detections[0].pose.covariance[0] = -1.0;
	detections_msg.detections[0].supports = detection_in_sensor.supports;
	detections_msg.detections[0].color.code = detection_in_sensor.detected_color;
	detections_publisher__.publish(detections_msg);

	if ( vizbose__ )
		publishMarkers(detections_msg);

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
	if ( T_sensor_to_robot__.contains(__header.frame_id) )
		return true;

	try
	{
		geometry_msgs::TransformStamped T_sensor_to_robot;
		T_sensor_to_robot = tf_buffer__.lookupTransform(robot_frame__, __header.frame_id, __header.stamp, ros::Duration(1.0));
		T_sensor_to_robot__[__header.frame_id] = T_sensor_to_robot;
	}
	catch (tf2::TransformException & ex)
	{
		ROS_WARN("%s", ex.what());
		ROS_WARN_STREAM("Error getting transform from " << robot_frame__ << " to " << __header.frame_id);
		return false;
	}

	ROS_INFO_STREAM("Static transform updated. From " << robot_frame__ << " to " << __header.frame_id);
	return true;
}

bool ColorInRoiPerceptor::parseColorCode(const std::string & __color_name, uint8_t & __color_code) const
{
	std::string normalized_color_name = __color_name;
	std::transform(
		normalized_color_name.begin(),
		normalized_color_name.end(),
		normalized_color_name.begin(),
		[](unsigned char __character) { return static_cast<char>(std::tolower(__character)); });

	if ( normalized_color_name == "red" )
	{
		__color_code = target_detector::Color::RED;
		return true;
	}
	if ( normalized_color_name == "green" )
	{
		__color_code = target_detector::Color::GREEN;
		return true;
	}
	if ( normalized_color_name == "blue" )
	{
		__color_code = target_detector::Color::BLUE;
		return true;
	}
	if ( normalized_color_name == "unknown" )
	{
		__color_code = target_detector::Color::UNKNOWN;
		return true;
	}

	return false;
}
}
