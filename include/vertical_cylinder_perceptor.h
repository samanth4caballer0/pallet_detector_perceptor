#ifndef TARGET_DETECTOR__VERTICAL_CYLINDER_PERCEPTOR_H
#define TARGET_DETECTOR__VERTICAL_CYLINDER_PERCEPTOR_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include "detectors/vertical_cylinder_detector.h"
#include <target_detector/DetectorEnable.h>
#include <target_detector/Detections.h>

namespace TargetDetector
{

class VerticalCylinderPerceptor
{
	protected:

		ros::NodeHandle nh__;
		std::string perceptor_name__;

		ros::Subscriber point_cloud_subscriber__;
		ros::Publisher detections_publisher__;
		ros::Publisher point_cloud_publisher__;
		ros::Publisher viz_markers_publisher__;
		ros::ServiceServer enable_server__;

		tf2_ros::Buffer tf_buffer__;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener__;
		std::map<std::string, geometry_msgs::TransformStamped> T_sensor_to_robot__;

		Detectors::VerticalCylinderDetector detector__;

		bool enabled__ = false;
		bool vizbose__ = false;
		double active_diameter__ = 0.0;
		double default_diameter__ = 0.0;
		std::string robot_frame__;
		std::string source_name__;
		int min_cloud_points__ = 1000;
		Eigen::Vector4f crop_max__;
		Eigen::Vector4f crop_min__;
		Eigen::Vector3f voxel_size__;

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

		bool validateDiameter(const double & __diameter) const;
};

} // end namespace

#endif
