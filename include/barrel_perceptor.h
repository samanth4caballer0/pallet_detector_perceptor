#ifndef DETECTORS__BARREL_PERCEPTOR_H
#define DETECTORS__BARREL_PERCEPTOR_H

//ROS dependencies
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h> //PCL-ROS interoperability
#include <pcl_conversions/pcl_conversions.h> //conversions from/to PCL/ROS
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

// this package
#include "detectors/detector_pcl_barrel.h" // custom detector
#include <target_detector/DetectorEnable.h> // ROS service to enable
#include <target_detector/Detections.h> // ROS message to publish detections

namespace TargetDetector
{

class BarrelPerceptor
{
    protected:

        //ros node handles
        ros::NodeHandle nh__;
		std::string perceptor_name__;

		// ROS interfaces
        ros::Subscriber point_cloud_subscriber__; // input cloud
		ros::Publisher detections_publisher__; // detection as a streaming
		ros::Publisher point_cloud_publisher__; // debugging and vizualizaton purposes
		ros::Publisher viz_markers_publisher__; // debugging and vizualizaton purposes
		ros::ServiceServer enable_server__;

		// custom detector
		Detectors::DetectorPclBarrel detector__;

		// node configs and management
		bool enabled__;
		bool vizbose__; // enable visualization flag
		bool enable_tf__; // enable tf broadcast
		std::string cloud_in_topic_name__; // name for the point cloud input topic

    public:
        //constructor
        BarrelPerceptor();

        //destructor
        ~BarrelPerceptor();

		//init
		bool init();

    protected:
		// Service to enable detector
		bool enableServiceCallback(
			target_detector::DetectorEnable::Request & __request,
			target_detector::DetectorEnable::Response & __response);

		// Point cloud callback
		void pointCloudCallback(
			const sensor_msgs::PointCloud2ConstPtr& __cloud_in);

}; //end of class

} //end of namespace

#endif
