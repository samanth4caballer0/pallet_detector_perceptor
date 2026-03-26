#include "detectors/color_in_roi_detector.h"

#include <algorithm>
#include <cmath>

namespace Detectors
{

bool ColorInRoiDetector::configure(
	const Eigen::Vector4f & __crop_min,
	const Eigen::Vector4f & __crop_max,
	const int & __min_color_inliers_points,
	const std::map<uint8_t, float> & __color_tag_to_hue)
{
	crop_min__ = __crop_min;
	crop_max__ = __crop_max;
	min_color_inliers_points__ = __min_color_inliers_points;
	color_tag_to_hue__.clear();
	for ( const auto & [color_code, hue_deg] : __color_tag_to_hue )
		color_tag_to_hue__[color_code] = normalizeHue(hue_deg);
	return true;
}

bool ColorInRoiDetector::detect(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr __cloud_in,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr __cloud_roi_out,
	ColorInRoiDetection & __detection_out) const
{
	if ( !__cloud_in || !__cloud_roi_out )
		return false;

	pcl::CropBox<pcl::PointXYZRGB> box_filter;
	box_filter.setMax(crop_max__);
	box_filter.setMin(crop_min__);
	box_filter.setInputCloud(__cloud_in);
	box_filter.setNegative(false);
	box_filter.filter(*__cloud_roi_out);

	if ( __cloud_roi_out->empty() )
		return false;

	struct ColorAccumulator
	{
		uint32_t supports = 0;
		Eigen::Vector3d position_sum = Eigen::Vector3d::Zero();
	};

	std::map<uint8_t, ColorAccumulator> accumulators;
	for ( const auto & [color_code, _] : color_tag_to_hue__ )
		accumulators[color_code] = ColorAccumulator{};

	const float hue_tolerance_deg = 20.0f;
	for ( const auto & roi_point : __cloud_roi_out->points )
	{
		if ( !std::isfinite(roi_point.x) || !std::isfinite(roi_point.y) || !std::isfinite(roi_point.z) )
			continue;

		const HSV hsv = rgbToHsv(roi_point.r, roi_point.g, roi_point.b);
		if ( hsv.s <= 0.2f )
			continue;

		for ( const auto & [color_code, hue_deg] : color_tag_to_hue__ )
		{
			if ( circularHueDistance(hsv.h, hue_deg) >= hue_tolerance_deg )
				continue;

			ColorAccumulator & accumulator = accumulators[color_code];
			accumulator.supports++;
			accumulator.position_sum.x() += roi_point.x;
			accumulator.position_sum.y() += roi_point.y;
			accumulator.position_sum.z() += roi_point.z;
		}
	}

	uint8_t detected_color = 0;
	uint32_t win_count = 0;
	for ( const auto & [color_code, accumulator] : accumulators )
	{
		if ( accumulator.supports > win_count )
		{
			detected_color = color_code;
			win_count = accumulator.supports;
		}
	}

	if ( detected_color == 0 || win_count < static_cast<uint32_t>(min_color_inliers_points__) )
		return false;

	const ColorAccumulator & winning_accumulator = accumulators.at(detected_color);
	const Eigen::Vector3d centroid = winning_accumulator.position_sum / static_cast<double>(winning_accumulator.supports);

	__detection_out.pose_in_sensor.position.x = centroid.x();
	__detection_out.pose_in_sensor.position.y = centroid.y();
	__detection_out.pose_in_sensor.position.z = centroid.z();
	__detection_out.pose_in_sensor.orientation.x = 0.0;
	__detection_out.pose_in_sensor.orientation.y = 0.0;
	__detection_out.pose_in_sensor.orientation.z = 0.0;
	__detection_out.pose_in_sensor.orientation.w = 1.0;
	__detection_out.detected_color = detected_color;
	__detection_out.supports = winning_accumulator.supports;
	return true;
}

float ColorInRoiDetector::normalizeHue(float __hue_deg) const
{
	float normalized_hue = std::fmod(__hue_deg, 360.0f);
	if ( normalized_hue < 0.0f )
		normalized_hue += 360.0f;
	return normalized_hue;
}

float ColorInRoiDetector::circularHueDistance(float __first_hue_deg, float __second_hue_deg) const
{
	const float normalized_first_hue = normalizeHue(__first_hue_deg);
	const float normalized_second_hue = normalizeHue(__second_hue_deg);
	const float hue_distance = std::fabs(normalized_first_hue - normalized_second_hue);
	return std::min(hue_distance, 360.0f - hue_distance);
}

ColorInRoiDetector::HSV ColorInRoiDetector::rgbToHsv(uint8_t __red, uint8_t __green, uint8_t __blue) const
{
	const float red = __red / 255.0f;
	const float green = __green / 255.0f;
	const float blue = __blue / 255.0f;

	const float max_value = std::max({red, green, blue});
	const float min_value = std::min({red, green, blue});
	const float delta = max_value - min_value;

	HSV hsv;
	hsv.v = max_value;
	hsv.s = (max_value == 0.0f) ? 0.0f : delta / max_value;

	if ( delta == 0.0f )
	{
		hsv.h = 0.0f;
		return hsv;
	}

	if ( max_value == red )
		hsv.h = 60.0f * std::fmod((green - blue) / delta, 6.0f);
	else if ( max_value == green )
		hsv.h = 60.0f * (((blue - red) / delta) + 2.0f);
	else
		hsv.h = 60.0f * (((red - green) / delta) + 4.0f);

	if ( hsv.h < 0.0f )
		hsv.h += 360.0f;
	return hsv;
}

} // namespace Detectors
