#ifndef DETECTORS__COLOR_IN_ROI_DETECTOR_H
#define DETECTORS__COLOR_IN_ROI_DETECTOR_H

#include <cstdint>
#include <map>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Detectors
{

struct ColorInRoiDetection
{
	geometry_msgs::Pose pose_in_sensor;
	uint8_t detected_color = 0;
	uint32_t supports = 0;
};

class ColorInRoiDetector
{
	protected:

		Eigen::Vector4f crop_max__;
		Eigen::Vector4f crop_min__;
		int min_color_inliers_points__ = 100;
		std::map<uint8_t, float> color_tag_to_hue__;

	public:

		bool configure(
			const Eigen::Vector4f & __crop_min,
			const Eigen::Vector4f & __crop_max,
			const int & __min_color_inliers_points,
			const std::map<uint8_t, float> & __color_tag_to_hue);

		bool detect(
			pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr __cloud_in,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr __cloud_roi_out,
			ColorInRoiDetection & __detection_out) const;

	protected:

		struct HSV
		{
			float h = 0.0f;
			float s = 0.0f;
			float v = 0.0f;
		};

		float normalizeHue(float __hue_deg) const;
		float circularHueDistance(float __first_hue_deg, float __second_hue_deg) const;
		HSV rgbToHsv(uint8_t __red, uint8_t __green, uint8_t __blue) const;
};

} // namespace Detectors

#endif
