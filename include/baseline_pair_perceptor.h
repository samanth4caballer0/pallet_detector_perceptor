#ifndef TARGET_DETECTOR__BASELINE_PAIR_PERCEPTOR_H
#define TARGET_DETECTOR__BASELINE_PAIR_PERCEPTOR_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <target_detector/DetectorEnable.h>
#include <target_detector/Detections.h>

namespace TargetDetector
{

class BaselinePairPerceptor
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
		double baseline_tolerance__ = 0.0;
		double baseline__ = 0.0;

	public:

		bool init();

	protected:

		bool enableCallback(target_detector::DetectorEnable::Request & __request, target_detector::DetectorEnable::Response & __response);
		bool configureParameters();

		void detectionsInCallback(const target_detector::Detections & __detections_in);
		void subscribeToData();
		void unsubscribeFromData();

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
			detection__.type = target_detector::Detection::BASELINE_PAIR;
			detection__.id = -1;
			detection__.pose.pose.position.z = 0.0;
			detection__.pose.pose.orientation.x = 0.0;
			detection__.pose.pose.orientation.y = 0.0;
			detection__.supports = 2;
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
			marker__.scale.x = 0.1;
			marker__.ns = perceptor_name__;
			marker__.type = visualization_msgs::Marker::LINE_LIST;
			marker__.color.r = 1.0;
			marker__.color.g = 0.6;
			marker__.color.b = 0.0;
			marker__.color.a = 0.75;
		};
};
	
}

#endif