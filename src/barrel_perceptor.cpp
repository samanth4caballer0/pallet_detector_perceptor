#include "barrel_perceptor.h"

namespace TargetDetector
{

BarrelPerceptor::BarrelPerceptor():
	nh__(),
	tf_listener__(tf_buffer__)
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
	verbose__ = true;
	vizbose__ = true; // enable visualization flag

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

	// Enabled by default (ONly for devel & debugging. )
	point_cloud_subscriber__ = nh__.subscribe("point_cloud_in", 1, &BarrelPerceptor::pointCloudCallback, this);
	enabled__ = true;

	// return
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

	// gets static transform from platform to camera
	if ( !getStaticTransform(__cloud_in->header.frame_id) )
	{
		ROS_WARN("BarrelPerceptor::pointCloudCallback(): Static transform from platform to camera not available.");
		return;
	}

	// local vars
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_crop(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_detection(new pcl::PointCloud<pcl::PointXYZ>());
	Eigen::Isometry3d T_O_C; // object wrt the camera (Check if better Affine3d ??)
	Eigen::Isometry3d T_O_P; // object wrt the platform
	double confidence_level;
	pcl::fromROSMsg (*__cloud_in, *cloud_in);

	// Crop to ROI
	Eigen::Vector4f crop_max(1.0, 1.0, 3.0, 1.0);
	Eigen::Vector4f crop_min(-1.0, 0.2, 0.5, 1.0);
	detector__.crop(crop_max, crop_min, cloud_in, cloud_crop);
	//point_cloud_publisher__.publish(cloud_crop);
	//return;

	// remove outliers
	//detector__.removeOutliers(cloud_crop, cloud_filtered);
	//point_cloud_publisher__.publish(cloud_filtered);
	//return;

	// downsampling
	detector__.voxelDownsampling(Eigen::Vector3f(0.02f,0.02f,0.02f), cloud_crop, cloud_downsampled);

	// detect barrel
	detector__.detect(0.5, cloud_downsampled, cloud_detection, T_O_C, confidence_level, vizbose__);
	//detector__.detect(0.5, cloud_filtered, cloud_detection, T_O_C, confidence_level, vizbose__);


	// Transform detection to the platform frame,
	T_O_P = T_platform_to_sensor__[__cloud_in->header.frame_id]*T_O_C;

	// fill ROS message & publish wrt to the platform
	target_detector::Detections detections_msg;
	Eigen::Quaterniond qt;
	detections_msg.header = __cloud_in->header;
	detections_msg.header.frame_id = "platform"; // wrt to the platform
	detections_msg.source_name = "camera";
	detections_msg.detections.resize(1);
	detections_msg.detections[0].type = target_detector::Detection::BARREL;
	detections_msg.detections[0].pose.pose.position.x = T_O_P.translation().x();
	detections_msg.detections[0].pose.pose.position.y = T_O_P.translation().y();
	detections_msg.detections[0].pose.pose.position.z = T_O_P.translation().z();
	detections_msg.detections[0].pose.covariance[0] = -1.;
	qt = Eigen::Quaterniond(T_O_P.linear());
	detections_msg.detections[0].pose.pose.orientation.x = qt.x();
	detections_msg.detections[0].pose.pose.orientation.y = qt.y();
	detections_msg.detections[0].pose.pose.orientation.z = qt.z();
	detections_msg.detections[0].pose.pose.orientation.w = qt.w();
	detections_msg.detections[0].intensity = 0.;
	detections_msg.detections[0].supports = cloud_detection->size();
	detections_publisher__.publish(detections_msg);

	// fill ROS message & publish wrt to the sensor
	detections_msg.header = __cloud_in->header; // keep frame_id , wrt to the camera
	detections_msg.source_name = "camera";
	detections_msg.detections.resize(1);
	detections_msg.detections[0].type = target_detector::Detection::BARREL;
	detections_msg.detections[0].pose.pose.position.x = T_O_C.translation().x();
	detections_msg.detections[0].pose.pose.position.y = T_O_C.translation().y();
	detections_msg.detections[0].pose.pose.position.z = T_O_C.translation().z();
	detections_msg.detections[0].pose.covariance[0] = -1.;
	qt = Eigen::Quaterniond(T_O_C.linear());
	detections_msg.detections[0].pose.pose.orientation.x = qt.x();
	detections_msg.detections[0].pose.pose.orientation.y = qt.y();
	detections_msg.detections[0].pose.pose.orientation.z = qt.z();
	detections_msg.detections[0].pose.pose.orientation.w = qt.w();
	detections_msg.detections[0].intensity = 0.;
	detections_msg.detections[0].supports = cloud_detection->size();
	//detections_publisher_sensor__.publish(detections_msg);

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

	marker_msg.header = __detections_msg.header;
	marker_msg.ns = "barrel_arrow";
    marker_msg.id = 0;
	marker_msg.type = visualization_msgs::Marker::ARROW;
	marker_msg.action = visualization_msgs::Marker::ADD;
	marker_msg.pose.position.x = __detections_msg.detections[0].pose.pose.position.x;
	marker_msg.pose.position.y = __detections_msg.detections[0].pose.pose.position.y + 0.5;
	marker_msg.pose.position.z = __detections_msg.detections[0].pose.pose.position.z;
	marker_msg.pose.orientation.x = __detections_msg.detections[0].pose.pose.orientation.x;
	marker_msg.pose.orientation.y = __detections_msg.detections[0].pose.pose.orientation.y;
	marker_msg.pose.orientation.z = __detections_msg.detections[0].pose.pose.orientation.z;
	marker_msg.pose.orientation.w = __detections_msg.detections[0].pose.pose.orientation.w;
	marker_msg.scale.x = 1.0;
	marker_msg.scale.y = 0.03;
	marker_msg.scale.z = 0.03;
	marker_msg.color.a = 0.8; // Don't forget to set the alpha!
	marker_msg.color.r = 1.0;
	marker_msg.color.g = 1.0;
	marker_msg.color.b = 0.0;
	viz_markers_publisher__.publish(marker_msg);

}

bool BarrelPerceptor::getStaticTransform(const std::string & __sensor_frame_id)
{
	// if transform alerady exists in the map, returns true
	if ( T_platform_to_sensor__.find(__sensor_frame_id) != T_platform_to_sensor__.end() )
	{
		return true;
	}

	// TF static. get sensor pose wrt the platform
	geometry_msgs::TransformStamped tr_st;
	try
	{
		tr_st = tf_buffer__.lookupTransform("platform", __sensor_frame_id, ros::Time(0), ros::Duration(1.));
 	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s",ex.what());
		ROS_WARN_STREAM("Error getting transform from " << "platform" << " to " << __sensor_frame_id);
		return false;
	}
    T_platform_to_sensor__[__sensor_frame_id].translation() <<
	        tr_st.transform.translation.x, tr_st.transform.translation.y, tr_st.transform.translation.z;
    Eigen::Quaterniond qq(
	        tr_st.transform.rotation.w, tr_st.transform.rotation.x, tr_st.transform.rotation.y, tr_st.transform.rotation.z);
    T_platform_to_sensor__[__sensor_frame_id].linear() = qq.toRotationMatrix();
	T_platform_to_sensor__[__sensor_frame_id].matrix().block<1,4>(3,0) << 0,0,0,1;
	if ( verbose__ )
	{
		std::cout 	<< "Transform from " << "platform" << " to " << __sensor_frame_id << ":" << std::endl
					<< T_platform_to_sensor__[__sensor_frame_id].matrix() << std::endl;
	}
	ROS_INFO_STREAM("Static transform updated. From " << "platform" << " to " << __sensor_frame_id);
	return true;
}

} // end of namespace
