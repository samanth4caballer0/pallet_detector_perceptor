#ifndef TARGET_DETECTOR__REFLECTOR_PERCEPTOR_H
#define TARGET_DETECTOR__REFLECTOR_PERCEPTOR_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <target_detector/DetectorEnable.h>
#include <target_detector/Detections.h>

#include <detectors/reflector_detector.h>

namespace TargetDetector
{

class ReflectorPerceptor
{
	protected:

		ros::NodeHandle nh__;
		std::string perceptor_name__;

		ros::Publisher detections_publisher__;
		target_detector::Detection detection__;

		ros::ServiceServer enable_server__;
		bool enabled__ = false;
		std::vector<ros::Subscriber> lidar_subscribers__;

		ros::Publisher markers_publisher__;
		bool vizbose__ = false;
		visualization_msgs::Marker marker__;

		double reflector_size__;
		double min_reflector_intensity__;
		double max_detection_range__;
		std::string robot_frame__;
		std::vector<std::string> lidars__;

		std::unique_ptr<Detectors::ReflectorDetector> detector__;

		tf2_ros::Buffer tf_buffer__;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener__;
		std::map<std::string, Eigen::Isometry3d> T_robot_to_sensor__;
		std::map<std::string, Eigen::Isometry2d> T_robot_to_sensor_2d__;

	public:

		bool init();

	protected:

		bool enableCallback(target_detector::DetectorEnable::Request & __request, target_detector::DetectorEnable::Response & __response);
		bool configureParameters();

		void laserScanCallback(const sensor_msgs::LaserScanConstPtr & __scan_ptr);
		void subscribeToLidars();
		void unsubscribeFromLidars();

		bool saveSensorTransform(const std_msgs::Header & __header);

		void publishMarkers(const target_detector::Detections & __detections);

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

		void initDetection()
		{
			detection__.type = target_detector::Detection::TYPE_REFLECTOR_FROM_INTENSITY;
			detection__.id = -1;
			detection__.pose.pose.position.z = 0.0;
			detection__.pose.pose.orientation.x = 0.0;
			detection__.pose.pose.orientation.y = 0.0;
			detection__.pose.pose.orientation.z = 0.0;
			detection__.pose.pose.orientation.w = 1.0;
		};

		void initMarker()
		{
			marker__.id = 0;
			marker__.action = visualization_msgs::Marker::ADD;
			marker__.lifetime = ros::Duration(0.5);
			marker__.pose.orientation.x = 0.0;
			marker__.pose.orientation.y = 0.0;
			marker__.pose.orientation.z = 0.0;
			marker__.pose.orientation.w = 1.0;
			marker__.scale.x = reflector_size__*7.5; // just increasing size to be visible. Typical sizes are 0.04, so we plot at 0.3
			marker__.scale.y = reflector_size__*7.5;
			marker__.scale.z = reflector_size__*7.5;
			marker__.ns = perceptor_name__;
			marker__.type = visualization_msgs::Marker::SPHERE_LIST;
			marker__.color.r = 1.0;
			marker__.color.g = 1.0;
			marker__.color.b = 0.0;
			marker__.color.a = 0.75;
		};
};
	
}

#endif