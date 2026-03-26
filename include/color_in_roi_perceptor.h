#ifndef TARGET_DETECTOR__COLOR_IN_ROI_PERCEPTOR_H
#define TARGET_DETECTOR__COLOR_IN_ROI_PERCEPTOR_H

//std C++
#include <iostream>
#include <algorithm>
#include <vector>
#include <map>
#include <cctype>

// EIGEN
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

// PCL
#include "detectors/color_in_roi_detector.h"

// DUNA
#include <target_detector/DetectorEnable.h>
#include <target_detector/Detections.h>

namespace TargetDetector
{

class ColorInRoiPerceptor
{
	protected:

		ros::NodeHandle nh__;
		std::string perceptor_name__;

		ros::Subscriber point_cloud_subscriber__;
		ros::Publisher detections_publisher__;
		ros::Publisher detections_sensor_frame_publisher__;
		ros::Publisher point_cloud_publisher__;
		ros::Publisher viz_markers_publisher__;
		ros::ServiceServer enable_server__;

		tf2_ros::Buffer tf_buffer__;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener__;
		std::map<std::string, geometry_msgs::TransformStamped> T_sensor_to_robot__;

		Detectors::ColorInRoiDetector detector__;

		bool enabled__ = false;
		bool publish_sensor_frame_detections__ = false;
		bool vizbose__ = false;
		std::string robot_frame__;
		std::string source_name__;
		uint8_t target_color__;
		Eigen::Vector4f crop_max__;
		Eigen::Vector4f crop_min__;
		int min_cloud_points__ = 1000;
		int min_color_inliers_points__ = 100;

	public:

		bool init();

	protected:

		bool configureParameters();

		bool enableServiceCallback(
			target_detector::DetectorEnable::Request & __request,
			target_detector::DetectorEnable::Response & __response);

		void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& __cloud_in);

		void subscribeToData();
		void unsubscribeFromData();

		void publishMarkers(const target_detector::Detections & __detections_msg);

		bool saveSensorTransform(const std_msgs::Header & __header);
		bool parseColorCode(const std::string & __color_name, uint8_t & __color_code) const;

		template <typename T>
		bool getParamOrFail(const std::string & __name, T& __variable)
		{
			if ( !nh__.getParam(__name, __variable) )
			{
				ROS_ERROR_STREAM("Failed to get parameter: " << __name);
				return false;
			}
			return true;
		};

}; // end of class

} //end of namespace

#endif
