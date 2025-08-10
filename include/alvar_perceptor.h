#ifndef TARGET_DETECTOR__ALVAR_PERCEPTOR_H
#define TARGET_DETECTOR__ALVAR_PERCEPTOR_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <target_detector/DetectorEnable.h>
#include <target_detector/Detections.h>

namespace TargetDetector
{

class AlvarPerceptor
{
	protected:

		ros::NodeHandle nh__;
		std::string perceptor_name__;

		ros::Publisher detections_out_publisher__;
		target_detector::Detection detection__;

		ros::ServiceServer enable_server__;
		bool enabled__ = false;
		ros::Subscriber detections_in_subscriber__;

		ros::Publisher markers_publisher__;
		bool vizbose__ = false;
		visualization_msgs::Marker marker__;

		std::string robot_frame__;
		int alvar_id__ = -1; // init to detect all

		double marker_size__;

		ros::Publisher enable_bundle_detection_publisher__;

		tf2_ros::Buffer tf_buffer__;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener__;
		std::map<std::string, Eigen::Isometry3d> T_robot_to_sensor__;
		std::map<std::string, Eigen::Isometry2d> T_robot_to_sensor_2d__;

	public:

		bool init();

	protected:

		bool enableCallback(target_detector::DetectorEnable::Request & __request, target_detector::DetectorEnable::Response & __response);
		bool configureParameters();

		void detectionsInCallback(const ar_track_alvar_msgs::AlvarMarkers & __bundles);
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

		void enableBundleDetection(const bool & __option)
		{
			std_msgs::Bool enable;
			enable.data = __option;
			enable_bundle_detection_publisher__.publish(enable);
		};

		void initDetection()
		{
			detection__.type = target_detector::Detection::TYPE_ALVAR;
			detection__.pose.pose.orientation.x = 0.0;
			detection__.pose.pose.orientation.y = 0.0;
		};

		void initMarker()
		{
			marker__.action = visualization_msgs::Marker::ADD;
			marker__.lifetime = ros::Duration(0.5);
			marker__.pose.orientation.x = 0.0;
			marker__.pose.orientation.y = 0.0;
			marker__.pose.orientation.z = 0.0;
			marker__.pose.orientation.w = 1.0;
			marker__.scale.x = 0.05;	// line width
			marker__.ns = perceptor_name__;
			marker__.type = visualization_msgs::Marker::LINE_LIST;
			marker__.color.r = 1.0;
			marker__.color.g = 0.4;
			marker__.color.b = 0.0;
			marker__.color.a = 0.75;
		};
};
	
}

#endif