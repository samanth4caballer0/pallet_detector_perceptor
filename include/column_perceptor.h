#ifndef TARGET_DETECTOR__COLUMN_PERCEPTOR_H
#define TARGET_DETECTOR__COLUMN_PERCEPTOR_H

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <target_detector/Detection.h>
#include <target_detector/DetectorEnable.h>
#include <target_detector/Detections.h>

#include "detectors/column_detector.h"

namespace TargetDetector
{

class ColumnPerceptor
{
	protected:

		ros::NodeHandle nh__;
		std::string perceptor_name__;

		ros::Publisher detections_publisher__;
		target_detector::Detection detection__;
		target_detector::Detections detections__;
		ros::Duration max_detection_age__ = ros::Duration(0.5);

		ros::ServiceServer enable_server__;
		bool enabled__ = false;
		std::vector<ros::Subscriber> lidar_subscribers__;
		std::map<std::string, unsigned int> scan_decimation_counter__;

		ros::Publisher markers_publisher__;
		bool vizbose__ = false;
		visualization_msgs::Marker marker__;
		std::map<std::string, int> sensor_ids__;

		double column_size__ = 0.5;
		double max_detection_range__ = 25.0;
		int decimation__ = 1;
		int override_support_points__ = 0;
		std::string robot_frame__;
		std::vector<std::string> lidars__;

		std::unique_ptr<Detectors::ColumnDetector> detector__;

		tf2_ros::Buffer tf_buffer__;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener__;
		std::map<std::string, geometry_msgs::TransformStamped> T_sensor_to_robot__;

	public:

		bool init();

	protected:

		bool enableCallback(target_detector::DetectorEnable::Request & __request, target_detector::DetectorEnable::Response & __response);
		bool configureParameters();

		void laserScanCallback(const sensor_msgs::LaserScanConstPtr & __scan_ptr, const std::string & __source_name);
		void processScan(
			const std_msgs::Header & __header,
			const std::string & __source_name,
			const double & __angle_min,
			const double & __angle_max,
			const std::vector<float> & __ranges);

		void subscribeToLidars();
		void unsubscribeFromLidars();

		bool saveSensorTransform(const std_msgs::Header & __header);
		void publishMarkers(const target_detector::Detections & __detections, const std::string & __source_name);

		template <typename T>
		bool getParamOrFail(const std::string & __name, T & __variable)
		{
			if ( !nh__.getParam(__name, __variable) )
			{
				ROS_ERROR_STREAM("Failed to get parameter: " << __name);
				return false;
			}
			return true;
		}

		void initDetection()
		{
			detection__.type = target_detector::Detection::COLUMN;
			detection__.id = -1;
			detection__.pose.pose.position.z = 0.0;
			detection__.pose.pose.orientation.x = 0.0;
			detection__.pose.pose.orientation.y = 0.0;
			detection__.pose.pose.orientation.z = 0.0;
			detection__.pose.pose.orientation.w = 1.0;
			detection__.intensity = 0.0;
			detection__.supports = 0;
			detection__.baseline = 0.0;
			detection__.radius = column_size__ / 2.0;
			detection__.points.clear();
		}

		void initDetections()
		{
			detections__.header.frame_id = robot_frame__;
		}

		void initMarker()
		{
			marker__.id = 0;
			marker__.action = visualization_msgs::Marker::ADD;
			marker__.lifetime = max_detection_age__;
			marker__.pose.orientation.x = 0.0;
			marker__.pose.orientation.y = 0.0;
			marker__.pose.orientation.z = 0.0;
			marker__.pose.orientation.w = 1.0;
			marker__.scale.x = column_size__;
			marker__.scale.y = column_size__;
			marker__.scale.z = column_size__;
			marker__.ns = perceptor_name__;
			marker__.type = visualization_msgs::Marker::SPHERE_LIST;
			marker__.color.r = 0.0;
			marker__.color.g = 1.0;
			marker__.color.b = 1.0;
			marker__.color.a = 0.75;
		}
};

}

#endif
