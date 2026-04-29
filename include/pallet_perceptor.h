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

#include <target_detector/DetectorEnable.h>
#include <target_detector/Detections.h>

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace TargetDetector
{

class PalletPerceptor
{
	protected:

		using PointT = pcl::PointXYZ;
		using CloudT = pcl::PointCloud<PointT>;

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

		// state
		bool enabled__ = false;
		bool vizbose__ = false;

		// standard params (shared with other perceptors)
		std::string robot_frame__;
		std::string source_name__;
		int min_cloud_points__ = 1000;
		Eigen::Vector4f crop_min__;
		Eigen::Vector4f crop_max__;
		Eigen::Vector3f voxel_size__;

		// Y-band crop in sensor optical frame (gravity assumption: Y=down)
		double floor_y__ = 0.55;

		// Euclidean clustering
		double cluster_tolerance__ = 0.05;
		int cluster_min_size__ = 500;
		int cluster_max_size__ = 200000;

		// expected pallet dimensions (EUR/EPAL)
		double pallet_width__ = 0.80;
		double pallet_height__ = 0.145;
		double tol_width__ = 0.10;
		double tol_height__ = 0.08;

		// pre-projection RANSAC for face-normal estimation
		int pre_ransac_max_iter__ = 200;
		double pre_ransac_distance_thresh__ = 0.04;
		int pre_ransac_min_inliers__ = 80;

		// template matching
		double tpl_cell_size__ = 0.02;
		double chi_threshold__ = 0.40;
		double tpl_stringer_width__ = 0.10;
		double tpl_top_deck_height__ = 0.04;
		double tpl_stringer_height__ = 0.06;

		// built-once template state
		std::vector<uint8_t> template_grid__;
		int template_cols__ = 0;
		int template_rows__ = 0;
		double template_mu__ = 0.0;

	public:

		bool init();

	protected:

		bool configureParameters();

		bool enableServiceCallback(
			target_detector::DetectorEnable::Request & __request,
			target_detector::DetectorEnable::Response & __response);

		void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & __cloud_in);

		void subscribeToData();
		void unsubscribeFromData();

		void publishMarkers(
			const target_detector::Detections & __detections_msg,
			const Eigen::Vector3f & __dims);

		bool saveSensorTransform(const std_msgs::Header & __header);

		// --- detection pipeline (per cluster) ---

		// Try to detect a pallet in a single cluster. On success: outputs the
		// object pose in sensor frame, OBB dims, the matched-window points, and chi.
		bool tryDetectPalletInCluster(
			CloudT::ConstPtr __cluster,
			Eigen::Isometry3d & __T_object_in_sensor,
			Eigen::Vector3f & __dims,
			CloudT::Ptr __matched_pts,
			double & __chi) const;

		// RANSAC vertical-plane fit on the cluster -> projection axis_u along the
		// pallet face width direction. Falls back to camera-X on failure.
		Eigen::Vector3f estimateFaceNormalAxisU(CloudT::ConstPtr __cluster) const;

		// Refine yaw and (x, z) face position from the matched-window points.
		// Returns true if RANSAC plane fit succeeded; false if PCA fallback was used.
		bool computePalletPose(
			CloudT::ConstPtr __matched_pts,
			float __match_y_lo,
			float __match_y_hi,
			float & __face_pos_x,
			float & __face_pos_z,
			float & __yaw) const;

		// --- template ---

		// Build EUR/EPAL end-face binary template (called once at init).
		void buildPalletTemplate();

		// Rasterize cluster points onto a 2D occupancy grid in face coords (u, v).
		void projectToFaceGrid(
			CloudT::ConstPtr __slice,
			const Eigen::Vector3f & __axis_u,
			const Eigen::Vector3f & __axis_v,
			std::vector<uint8_t> & __grid,
			int & __gcols,
			int & __grows,
			float & __u_min,
			float & __v_min) const;

		// Slide the pre-built template, return best chi score and matched window.
		double slideTemplate(
			const std::vector<uint8_t> & __grid,
			int __gcols,
			int __grows,
			int & __best_co,
			int & __best_ro) const;

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
