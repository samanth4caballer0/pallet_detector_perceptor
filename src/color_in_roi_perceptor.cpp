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
	std::vector<double> voxel_size;
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

	color_tag_to_hue__[target_detector::Color::RED] = normalizeHue(static_cast<float>(red_hue_center_deg));
	color_tag_to_hue__[target_detector::Color::GREEN] = normalizeHue(static_cast<float>(green_hue_center_deg));
	color_tag_to_hue__[target_detector::Color::BLUE] = normalizeHue(static_cast<float>(blue_hue_center_deg));

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

	// crop
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_crop(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(*__cloud_in, *cloud_in);
	pcl::CropBox<pcl::PointXYZRGB> box_filter;
	box_filter.setMax(crop_max__);
	box_filter.setMin(crop_min__);
	box_filter.setInputCloud(cloud_in);
	box_filter.setNegative(false);
	box_filter.filter(*cloud_crop);
	point_cloud_publisher__.publish(cloud_crop);

	if ( cloud_crop->empty() )
		return;

	// detect color
	unsigned int red_count = 0;
	unsigned int green_count = 0;
	unsigned int blue_count = 0;
	unsigned int win_count = 0;
	uint8_t detected_color = target_detector::Color::UNKNOWN;
	const float hue_tolerance_deg = 20.0f;
	for ( const auto & roi_point : cloud_crop->points )
	{
		HSV hsv = rgbToHsv(roi_point.r, roi_point.g, roi_point.b);
		//std::cout << "Point " << ii << " -> H:" << hsv.h << " S:" << hsv.s << " V:" << hsv.v << std::endl;
		if ( hsv.s > 0.2 )
		{
			if ( circularHueDistance(hsv.h, color_tag_to_hue__.at(target_detector::Color::RED)) < hue_tolerance_deg )
				red_count ++;
			if ( circularHueDistance(hsv.h, color_tag_to_hue__.at(target_detector::Color::GREEN)) < hue_tolerance_deg )
				green_count ++;
			if ( circularHueDistance(hsv.h, color_tag_to_hue__.at(target_detector::Color::BLUE)) < hue_tolerance_deg )
				blue_count ++;
		}
	}
	if ( ( red_count >= green_count ) && ( red_count >= blue_count ) )
	{
		detected_color = target_detector::Color::RED;
		win_count = red_count;
	}
	if ( ( green_count > red_count ) && ( green_count >= blue_count ) )
	{
		detected_color = target_detector::Color::GREEN;
		win_count = green_count;
	}
	if ( ( blue_count > red_count ) && ( blue_count > green_count ) )
	{
		detected_color = target_detector::Color::BLUE;
		win_count = blue_count;
	}
	//std::cout << "Detected color " << (unsigned int)detected_color << "; counts (rgb): " << red_count << "," << green_count << "," << blue_count << std::endl;

	// If positive detection, fill ROS message
	if ( ( win_count >= static_cast<unsigned int>(min_color_inliers_points__) ) && ( detected_color == target_color__ ) )
	{
		target_detector::Detections detections_msg;
		detections_msg.header = __cloud_in->header;
		detections_msg.header.frame_id = robot_frame__;
		detections_msg.source_name = source_name__;
		detections_msg.detections.resize(1);
		detections_msg.detections[0].type = target_detector::Detection::COLOR_IN_ROI;
		detections_msg.detections[0].pose.pose.position.x = 0.5*(crop_min__.x()+crop_max__.x());
		detections_msg.detections[0].pose.pose.position.y = 0.5*(crop_min__.y()+crop_max__.y());
		detections_msg.detections[0].pose.pose.position.z = 0.5*(crop_min__.z()+crop_max__.z());
		detections_msg.detections[0].pose.covariance[0] = -1.0;
		detections_msg.detections[0].pose.pose.orientation.x = 0.0;
		detections_msg.detections[0].pose.pose.orientation.y = 0.0;
		detections_msg.detections[0].pose.pose.orientation.z = 0.0;
		detections_msg.detections[0].pose.pose.orientation.w = 1.0;
		detections_msg.detections[0].supports = win_count;
		detections_msg.detections[0].color.code = detected_color;
		detections_publisher__.publish(detections_msg);
	}

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

float ColorInRoiPerceptor::normalizeHue(float __hue_deg) const
{
	float normalized_hue = std::fmod(__hue_deg, 360.0f);
	if ( normalized_hue < 0.0f )
		normalized_hue += 360.0f;
	return normalized_hue;
}

float ColorInRoiPerceptor::circularHueDistance(float __first_hue_deg, float __second_hue_deg) const
{
	const float normalized_first_hue = normalizeHue(__first_hue_deg);
	const float normalized_second_hue = normalizeHue(__second_hue_deg);
	const float hue_distance = std::fabs(normalized_first_hue - normalized_second_hue);
	return std::min(hue_distance, 360.0f - hue_distance);
}

HSV ColorInRoiPerceptor::rgbToHsv(uint8_t r, uint8_t g, uint8_t b)
{
	// Normalize to [0,1]
	float rf = r / 255.0f;
	float gf = g / 255.0f;
	float bf = b / 255.0f;

	float max = std::max({rf, gf, bf});
	float min = std::min({rf, gf, bf});
	float delta = max - min;

	HSV hsv;

	// Value
	hsv.v = max;

	// Saturation
	if ( max == 0.0f )
		hsv.s = 0.0f;
	else
		hsv.s = delta / max;

	// Hue
	if ( delta == 0.0f )
	{
		hsv.h = 0.0f; // undefined, achromatic
	}
	else
	{
		if ( max == rf )
			hsv.h = 60.0f * (fmod(((gf - bf) / delta), 6.0f));
		else if ( max == gf )
			hsv.h = 60.0f * (((bf - rf) / delta) + 2.0f);
		else
			hsv.h = 60.0f * (((rf - gf) / delta) + 4.0f); // max == bf

		if ( hsv.h < 0.0f )
			hsv.h += 360.0f;
	}

	return hsv;
}

}
