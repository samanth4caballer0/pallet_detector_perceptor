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

	color_tag_to_hue__[target_detector::Color::RED] = 0.0;
	color_tag_to_hue__[target_detector::Color::GREEN] = 120.0;
	color_tag_to_hue__[target_detector::Color::BLUE] = 240.0;

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
	int t_color;
	if ( !getParamOrFail("enabled_by_default", enabled__) ||
		!getParamOrFail("vizbose", vizbose__) ||
		!getParamOrFail("robot_frame", robot_frame__) ||
		!getParamOrFail("source_name", source_name__) ||
		!getParamOrFail("default_color", t_color) ||
		!getParamOrFail("crop_min", crop_min) ||
		!getParamOrFail("crop_max", crop_max) ||
		!getParamOrFail("min_cloud_points", min_cloud_points__) ||
		!getParamOrFail("min_color_inliers_percentage", min_color_inliers_percentage__) )
	{
		return false;
	}

	if ( min_cloud_points__ < 1 )
	{
		ROS_ERROR_STREAM("Invalid parameter min_cloud_points: " << min_cloud_points__ << ". It must be >= 1.");
		return false;
	}

	if ( ( min_color_inliers_percentage__ < 0 ) || ( min_color_inliers_percentage__ > 100 ) )
	{
		ROS_ERROR_STREAM("Invalid parameter min_color_inliers_percentage: " << min_color_inliers_percentage__ << ". It must be in [0,100]");
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

	target_color__ = static_cast<uint8_t>(t_color);
	std::cout << "target_color: " << (unsigned int)target_color__ << std::endl;

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

	// detect color
	unsigned int red_count = 0;
	unsigned int green_count = 0;
	unsigned int blue_count = 0;
	unsigned int win_count;
	uint8_t detected_color;
	for (unsigned int ii=0; ii<cloud_crop->points.size(); ii++)
	{
		HSV hsv = rgbToHsv(cloud_crop->at(ii).r, cloud_crop->at(ii).g, cloud_crop->at(ii).b);
		//std::cout << "Point " << ii << " -> H:" << hsv.h << " S:" << hsv.s << " V:" << hsv.v << std::endl;
		if ( hsv.s > 0.2 )
		{
			if ( std::fabs( hsv.h - 0.0 ) < 20.0 ) red_count ++;
			if ( std::fabs( hsv.h - 360.0 ) < 20.0 ) red_count ++;
			if ( std::fabs( hsv.h - 120.0 ) < 20.0 ) green_count ++;
			if ( std::fabs( hsv.h - 240.0 ) < 20.0 ) blue_count ++;
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
	std::cout << "Detected color " << (unsigned int)detected_color << "; counts (rgb): " << red_count << "," << green_count << "," << blue_count << std::endl;

	// If positive detection, fill ROS message
	if ( detected_color == target_color__ )
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

/*
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

	// Find RGB field offset
	uint32_t x_offset = -1, y_offset = -1, z_offset = -1;
    uint32_t rgb_offset = -1;
    uint32_t r_offset = -1, g_offset = -1, b_offset = -1;
    for (const auto& field : __cloud_in->fields)
    {
		if (field.name == "x") x_offset = field.offset;
		if (field.name == "y") y_offset = field.offset;
		if (field.name == "z") z_offset = field.offset;
        if (field.name == "rgb") rgb_offset = field.offset;
        if (field.name == "r") r_offset = field.offset;
        if (field.name == "g") g_offset = field.offset;
        if (field.name == "b") b_offset = field.offset;
    }

	// loop over all points. Apply ROI and get color
	float xx,yy,zz,rgb;
	uint8_t rr,gg,bb;
	uint32_t rgb_int;
    for (int ii=0; ii< __cloud_in->height; ii++)
    {
        for (int jj=0; jj< __cloud_in->width; jj++)
        {
			// point index
            size_t point_ij = ii * __cloud_in->width + jj;
            size_t byte_ij = point_ij * __cloud_in->point_step;

			// First check ROI, then get color
			xx = *reinterpret_cast<const float*>(&__cloud_in->data[byte_ij + x_offset]);
			yy = *reinterpret_cast<const float*>(&__cloud_in->data[byte_ij + y_offset]);
			zz = *reinterpret_cast<const float*>(&__cloud_in->data[byte_ij + z_offset]);
			if (	xx > crop_min__.x() && xx < crop_max__.x() &&
					yy > crop_min__.y() && yy < crop_max__.y() &&
					zz > crop_min__.z() && zz < crop_max__.z()    )
			{
				// COLOR
	            if (rgb_offset >= 0)
	            {
	                rgb = *reinterpret_cast<const float*>(&__cloud_in->data[byte_ij + rgb_offset]);
	                rgb_int = *reinterpret_cast<uint32_t*>(&rgb);
	                rr = (rgb_int >> 16) & 0xFF;
	                gg = (rgb_int >> 8)  & 0xFF;
	                bb = rgb_int & 0xFF;
	            }
	            else
	            {
	                rr = __cloud_in->data[byte_ij + r_offset];
	                gg = __cloud_in->data[byte_ij + g_offset];
	                bb = __cloud_in->data[byte_ij + b_offset];
	            }
				HSV hsv = rgbToHsv(rr, gg, bb);
				std::cout << "Point (" << xx << "," << yy << "," << zz << ") -> H:" << hsv.h << " S:" << hsv.s << " V:" << hsv.v << std::endl;
			}
        }
    }
}
*/

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
    if (max == 0.0f)
        hsv.s = 0.0f;
    else
        hsv.s = delta / max;

    // Hue
    if (delta == 0.0f) {
        hsv.h = 0.0f; // undefined, achromatic
    } else {
        if (max == rf) {
            hsv.h = 60.0f * (fmod(((gf - bf) / delta), 6.0f));
        } else if (max == gf) {
            hsv.h = 60.0f * (((bf - rf) / delta) + 2.0f);
        } else { // max == bf
            hsv.h = 60.0f * (((rf - gf) / delta) + 4.0f);
        }

        if (hsv.h < 0.0f)
            hsv.h += 360.0f;
    }

    return hsv;
}

} // namespace TargetDetector
