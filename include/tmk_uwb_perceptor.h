#ifndef TARGET_DETECTOR__TMK_UWB_PERCEPTOR_H
#define TARGET_DETECTOR__TMK_UWB_PERCEPTOR_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <tmk_uwb/UwbMeasurement.h>

#include <target_detector/DetectorEnable.h>
#include <target_detector/Detections.h>

namespace TargetDetector
{

class TmkUwbPerceptor
{
	protected:

		ros::NodeHandle nh__;
		std::string perceptor_name__;

		ros::Publisher detections_out_publisher__;
		target_detector::Detection detection__;
		target_detector::Detections detections__;

		ros::ServiceServer enable_server__;
		bool enabled__ = false;
		ros::Subscriber detections_in_subscriber__;

		ros::Publisher markers_publisher__;
		bool vizbose__ = false;
		visualization_msgs::Marker marker__;

		std::string robot_frame__;

		tf2_ros::Buffer tf_buffer__;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener__;
		std::map<std::string, geometry_msgs::TransformStamped> T_sensor_to_robot__; // T from sensor to robot

	public:

		bool init();

	protected:

		bool enableCallback(target_detector::DetectorEnable::Request & __request, target_detector::DetectorEnable::Response & __response);
		bool configureParameters();

		void detectionsInCallback(const tmk_uwb::UwbMeasurement & __msg);
		void subscribeToData();
		void unsubscribeFromData();

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
			detection__.type = target_detector::Detection::TMK_UWB;
			detection__.pose.pose.orientation.x = 0.0;
			detection__.pose.pose.orientation.y = 0.0;
		};
		void initDetections()
		{
			detections__.header.frame_id = robot_frame__;
		};

		void initMarker()
		{
			marker__.action = visualization_msgs::Marker::ADD;
			marker__.lifetime = ros::Duration(0.5);
			marker__.pose.orientation.x = 0.0;
			marker__.pose.orientation.y = 0.0;
			marker__.pose.orientation.z = 0.0;
			marker__.pose.orientation.w = 1.0;
			marker__.scale.x = 0.25;	// arrow length
			marker__.scale.y = 0.05;	// arrow width
			marker__.scale.z = 0.05;	// arrow height
			marker__.ns = perceptor_name__;
			marker__.type = visualization_msgs::Marker::ARROW;
			marker__.color.r = 1.0;
			marker__.color.g = 0.8;
			marker__.color.b = 0.0;
			marker__.color.a = 0.75;
		};
};
	
}

#endif
