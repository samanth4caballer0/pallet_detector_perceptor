#ifndef TARGET_DETECTOR__PALLET_PERCEPTOR_H
#define TARGET_DETECTOR__PALLET_PERCEPTOR_H

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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "detectors/pallet_detector.h"
#include <target_detector/DetectorEnable.h>
#include <target_detector/Detections.h>

#include <cstdint>
#include <map>
#include <memory>
#include <string>

namespace TargetDetector
{

class PalletPerceptor
{
	protected:

		using PointT = pcl::PointXYZ;
		using CloudT = pcl::PointCloud<PointT>;

		ros::NodeHandle nh__;
		std::string perceptor_name__;

		ros::Subscriber    point_cloud_subscriber__;
		ros::Publisher     detections_publisher__;
		ros::Publisher     point_cloud_publisher__;        // cloud_out (matched_pts of best)
		ros::Publisher     ransac_inliers_publisher__;     // cloud_ransac_inliers (debug)
		ros::Publisher     viz_markers_publisher__;        // visuals (both cube markers)
		ros::Publisher     no_plane_cloud_publisher__;
		ros::Publisher     downsampled_cloud_publisher__;
		ros::Publisher     pose_publisher__;
		ros::ServiceServer enable_server__;

		tf2_ros::Buffer tf_buffer__;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener__;
		std::map<std::string, geometry_msgs::TransformStamped> T_sensor_to_robot__;

		// The pure detection algorithm. Configured at init() from ROS params.
		Detectors::PalletDetector detector__;

		// State
		bool enabled__ = false;
		bool vizbose__ = false;

		// Sensor-frame preprocessing params (live in the perceptor; the detector is sensor-agnostic)
		std::string robot_frame__;
		std::string source_name__;
		int    min_cloud_points__ = 1000;
		Eigen::Vector3f voxel_size__;
		double floor_y__         = 0.27;  // Y of floor in camera optical frame (gravity)
		double pallet_height__   = 0.145;
		double tol_height__      = 0.08;

	public:

		bool init();

		// Prints the cluster-level rejection breakdown on destruction so the
		// Ch5 Limitations section can quote it directly from the launch log.
		~PalletPerceptor();

	protected:

		bool configureParameters();

		bool enableServiceCallback(
			target_detector::DetectorEnable::Request & __request,
			target_detector::DetectorEnable::Response & __response);

		void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & __cloud_in);

		void subscribeToData();
		void unsubscribeFromData();

		// Publish the OBB cube (matched_pts AABB, green) and the RANSAC-inlier
		// cube (front-face plane AABB, red), both in sensor frame.
		void publishMarkers(
			const std_msgs::Header & __header,
			const geometry_msgs::Pose & __pose_in_sensor,
			const Eigen::Vector3f & __obb_dims,
			const geometry_msgs::Pose & __ransac_pose_in_sensor,
			const Eigen::Vector3f & __ransac_dims);

		bool saveSensorTransform(const std_msgs::Header & __header);

		template <typename T>
		bool getParamOrFail(const std::string & __name, T & __variable)
		{
			if ( !nh__.getParam(__name, __variable) )
			{
				ROS_ERROR_STREAM("Failed to get parameter: " << __name);
				return false;
			}
			return true;
		};
};

} // end namespace

#endif
