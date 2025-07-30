#ifndef TARGET_DETECTOR__REFLECTOR_PERCEPTOR_H
#define TARGET_DETECTOR__REFLECTOR_PERCEPTOR_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>

#include <target_detector/DetectorEnable.h>
#include <target_detector/Detections.h>

#include <detectors/reflector_detector.h>

namespace TargetDetector
{

class ReflectorPerceptor
{
	protected:

		ros::NodeHandle nh__;

		ros::Publisher detections_publisher__;
		ros::ServiceServer enable_server__;

		std::vector<ros::Subscriber> lidar_subscribers__;

		bool enabled__;
		double reflector_size__;
		double min_reflector_intensity__;
		double max_detection_range__;
		std::string robot_frame__;
		std::vector<std::string> lidars__;

		std::unique_ptr<ReflectorDetector> detector__;

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

};
	
}

#endif