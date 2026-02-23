#include "barrel_perceptor.h"

namespace TargetDetector
{

BarrelPerceptor::BarrelPerceptor():
	nh__()
{
	//
}

BarrelPerceptor::~BarrelPerceptor()
{
	//
}

bool BarrelPerceptor::init()
{

	// Get config params (TODO, get them from yaml)
	enabled__ = false;
	vizbose__ = true; // enable visualization flag
	enable_tf__ = false; // enable tf broadcast
	cloud_in_topic_name__ = "camera";

	// ROS API
	detections_publisher__ = nh__.advertise<target_detector::Detections>("detections", 1, false);
	enable_server__ = nh__.advertiseService("enable", &BarrelPerceptor::enableServiceCallback, this);
	if ( vizbose__ )
	{
		viz_markers_publisher__ = nh__.advertise<visualization_msgs::Marker>("visuals", 1, false);
	}

	// init detector
	if ( !detector__.init() )
	{
		ROS_ERROR("BarrelPerceptor::init(): detector not initialized correctly. Exit");
		return false;
	}

	return true;
};

bool BarrelPerceptor::enableServiceCallback(
	target_detector::DetectorEnable::Request & __request,
	target_detector::DetectorEnable::Response & __response)
{
	// enable
	if ( !enabled__ && __request.enable )
	{
		point_cloud_subscriber__ = nh__.subscribe("point_cloud_in", 1, &BarrelPerceptor::pointCloudCallback, this);
	}

	// disable
	if ( enabled__ && !__request.enable )
	{
		point_cloud_subscriber__.shutdown();

		// since we disable, make sure to publish an empty detections message to clear interested parties
		target_detector::Detections msg;
		msg.header.stamp = ros::Time::now();
		detections_publisher__.publish(msg);
	}

	// return
	enabled__ = __request.enable;
	__response.success = true;
	return true;
}

void BarrelPerceptor::pointCloudCallback(
	const sensor_msgs::PointCloud2ConstPtr& __cloud_in)
{
	// basic checks
	if ( !enabled__ ) return;
	if (__cloud_in.get() == nullptr) // input cloud existence
	{
		ROS_WARN("BarrelPerceptor::pointCloudCallback(): Void input cloud ");
		return;
	}
	if (__cloud_in->data.size() < 1000) //input cloud enough populated
	{
		ROS_WARN("BarrelPerceptor::pointCloudCallback(): Too few points on the input cloud ");
		return;
	}

	// local vars
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_crop(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_detection(new pcl::PointCloud<pcl::PointXYZ>());
	Eigen::Isometry3d T_O_C; // object wrt the camera (Check if better Affine3d ??)
	double confidence_level;
	pcl::fromROSMsg (*__cloud_in, *cloud_in);

	// Crop to ROI
	Eigen::Vector4f crop_max(3.0,  1.5, 0.0, 1.0);
	Eigen::Vector4f crop_min(0.5, -1.5, -1.0, 1.0);
	detector__.crop(crop_max, crop_min, cloud_in, cloud_crop);

	// remove outliers
	detector__.removeOutliers(cloud_crop, cloud_filtered);
	//point_cloud_publisher__.publish(cloud_filtered);

	// detect barrel
	detector__.detect(1.0, cloud_filtered, cloud_detection, T_O_C, confidence_level, vizbose__);

	// fill ROS message & publish
	target_detector::Detections detections_msg;
	detections_msg.header = __cloud_in->header;
	detections_msg.source_name = "camera";
	detections_msg.detections.resize(1);
	detections_msg.detections[0].pose.pose.position.x = T_O_C.translation().x();
	detections_msg.detections[0].pose.pose.position.y = T_O_C.translation().y();
	detections_msg.detections[0].pose.pose.position.z = T_O_C.translation().z();
	detections_msg.detections[0].pose.covariance[0] = -1.;
	detections_msg.detections[0].pose.pose.orientation.x = 0.;
	detections_msg.detections[0].pose.pose.orientation.y = 0.;
	detections_msg.detections[0].pose.pose.orientation.z = 1.; //always pointing to the sensor
	detections_msg.detections[0].pose.pose.orientation.w = 0.; //always pointing to the sensor
	detections_msg.detections[0].intensity = 0.;
	detections_msg.detections[0].supports = cloud_detection->size();
	detections_publisher__.publish(detections_msg);

	// fill vizualizations and publish
	if ( vizbose__ )
	{
		point_cloud_publisher__.publish(cloud_detection);
		//publishMarkers(...);
	}
}

} // end of namespace
