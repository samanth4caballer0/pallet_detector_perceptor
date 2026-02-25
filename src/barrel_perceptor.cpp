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
	enable_server__ = nh__.advertiseService("enable", &BarrelPerceptor::enableServiceCallback, this);
	detections_publisher__ = nh__.advertise<target_detector::Detections>("detections", 1, false);
	point_cloud_publisher__ = nh__.advertise<sensor_msgs::PointCloud2>("cloud_out", 1, false);
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_detection(new pcl::PointCloud<pcl::PointXYZ>());
	Eigen::Isometry3d T_O_C; // object wrt the camera (Check if better Affine3d ??)
	double confidence_level;
	pcl::fromROSMsg (*__cloud_in, *cloud_in);

	// Crop to ROI
	Eigen::Vector4f crop_max(1.0,  1.0, 3.0, 1.0);
	Eigen::Vector4f crop_min(-1.0, 0.2, 0.5, 1.0);
	detector__.crop(crop_max, crop_min, cloud_in, cloud_crop);
	//point_cloud_publisher__.publish(cloud_crop);
	//return;

	// remove outliers
	//detector__.removeOutliers(cloud_crop, cloud_filtered);
	//point_cloud_publisher__.publish(cloud_filtered);
	//return;

	// downsampling
	detector__.voxelDownsampling(cloud_crop, cloud_downsampled);

	// detect barrel
	detector__.detect(0.5, cloud_downsampled, cloud_detection, T_O_C, confidence_level, vizbose__);
	//detector__.detect(0.5, cloud_filtered, cloud_detection, T_O_C, confidence_level, vizbose__);

	// fill ROS message & publish
	target_detector::Detections detections_msg;
	detections_msg.header = __cloud_in->header;
	detections_msg.source_name = "camera";
	detections_msg.detections.resize(1);
	detections_msg.detections[0].type = target_detector::Detection::BARREL; 
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
		publishMarkers(detections_msg);
	}
}

void BarrelPerceptor::publishMarkers(
	const target_detector::Detections & __detections_msg)
{
	visualization_msgs::Marker marker_msg;
	marker_msg.header = __detections_msg.header;
	marker_msg.ns = "barrel";
    marker_msg.id = 0;
	marker_msg.type = visualization_msgs::Marker::CYLINDER;
	marker_msg.action = visualization_msgs::Marker::ADD;
	marker_msg.pose.position.x = __detections_msg.detections[0].pose.pose.position.x;
	marker_msg.pose.position.y = __detections_msg.detections[0].pose.pose.position.y+0.5;
	marker_msg.pose.position.z = __detections_msg.detections[0].pose.pose.position.z;
	marker_msg.pose.orientation.x = std::sin(M_PI/4.);
	marker_msg.pose.orientation.y = 0.0;
	marker_msg.pose.orientation.z = 0.0;
	marker_msg.pose.orientation.w = std::cos(M_PI/4.);
	marker_msg.scale.x = 1.0;
	marker_msg.scale.y = 1.0;
	marker_msg.scale.z = 1.0;
	marker_msg.color.a = 0.6; // Don't forget to set the alpha!
	marker_msg.color.r = 1.0;
	marker_msg.color.g = 1.0;
	marker_msg.color.b = 0.0;
	viz_markers_publisher__.publish(marker_msg);
}

} // end of namespace
