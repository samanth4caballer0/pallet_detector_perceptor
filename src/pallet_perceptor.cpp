#include "pallet_perceptor.h"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace TargetDetector
{

namespace
{
	// ---- Algorithm constants (not exposed as user params) ----
	constexpr float kPreRansacEpsAngleRad   = 0.44f;  // ~25 deg, accepts front face up to 25 deg yaw
	constexpr float kPoseRansacEpsAngleRad  = 0.785f; // ~45 deg, more permissive for stringer fit
	constexpr float kPoseRansacDistThresh   = 0.02f;
	constexpr int   kPoseRansacMaxIter      = 100;
	constexpr int   kPoseRansacMinInliers   = 30;
	constexpr float kVerticalNormalThresh   = 0.5f;   // |n.y| < this for the plane to count as vertical
	constexpr float kFloorBandMargin        = 0.02f;  // exclude floor by 2 cm
	constexpr int   kMinClusterSize         = 50;
	constexpr int   kMinMatchedPoints       = 30;
	constexpr int   kMinPostBandPoints      = 200;
	constexpr int   kMinPostVoxelPoints     = 100;
	constexpr float kObbDepthForViz         = 0.05f;
}

// ---------------------------------------------------------------------------
// Init / configuration
// ---------------------------------------------------------------------------

bool PalletPerceptor::init()
{
	if ( !configureParameters() )
	{
		ROS_ERROR("Failed to configure pallet detection");
		return false;
	}

	buildPalletTemplate();

	tf_listener__.reset(new tf2_ros::TransformListener(tf_buffer__));

	enable_server__       = nh__.advertiseService("enable", &PalletPerceptor::enableServiceCallback, this);
	detections_publisher__ = nh__.advertise<target_detector::Detections>("detections", 1, false);

	if ( enabled__ )
		subscribeToData();

	if ( vizbose__ )
	{
		point_cloud_publisher__  = nh__.advertise<sensor_msgs::PointCloud2>("cloud_out", 1, false);
		viz_markers_publisher__  = nh__.advertise<visualization_msgs::Marker>("visuals", 1, false);
	}

	return true;
}

bool PalletPerceptor::configureParameters()
{
	perceptor_name__ = ros::this_node::getNamespace().substr(ros::this_node::getNamespace().find_last_of('/') + 1);

	std::vector<double> crop_min, crop_max, voxel_size;

	if ( !getParamOrFail("enabled_by_default",         enabled__) ||
		 !getParamOrFail("vizbose",                    vizbose__) ||
		 !getParamOrFail("robot_frame",                robot_frame__) ||
		 !getParamOrFail("source_name",                source_name__) ||
		 !getParamOrFail("min_cloud_points",           min_cloud_points__) ||
		 !getParamOrFail("crop_min",                   crop_min) ||
		 !getParamOrFail("crop_max",                   crop_max) ||
		 !getParamOrFail("voxel_size",                 voxel_size) ||
		 !getParamOrFail("floor_y",                    floor_y__) ||
		 !getParamOrFail("cluster_tolerance",          cluster_tolerance__) ||
		 !getParamOrFail("cluster_min_size",           cluster_min_size__) ||
		 !getParamOrFail("cluster_max_size",           cluster_max_size__) ||
		 !getParamOrFail("pallet_width",               pallet_width__) ||
		 !getParamOrFail("pallet_height",              pallet_height__) ||
		 !getParamOrFail("tol_width",                  tol_width__) ||
		 !getParamOrFail("tol_height",                 tol_height__) ||
		 !getParamOrFail("pre_ransac_max_iter",        pre_ransac_max_iter__) ||
		 !getParamOrFail("pre_ransac_distance_thresh", pre_ransac_distance_thresh__) ||
		 !getParamOrFail("pre_ransac_min_inliers",     pre_ransac_min_inliers__) ||
		 !getParamOrFail("tpl_cell_size",              tpl_cell_size__) ||
		 !getParamOrFail("chi_threshold",              chi_threshold__) ||
		 !getParamOrFail("tpl_stringer_width",         tpl_stringer_width__) ||
		 !getParamOrFail("tpl_top_deck_height",        tpl_top_deck_height__) ||
		 !getParamOrFail("tpl_stringer_height",        tpl_stringer_height__) )
	{
		return false;
	}

	if ( crop_min.size() != 3 || crop_max.size() != 3 || voxel_size.size() != 3 )
	{
		ROS_ERROR("crop_min, crop_max, voxel_size must each have 3 elements.");
		return false;
	}

	crop_min__   = Eigen::Vector4f(crop_min[0], crop_min[1], crop_min[2], 1.0f);
	crop_max__   = Eigen::Vector4f(crop_max[0], crop_max[1], crop_max[2], 1.0f);
	voxel_size__ = Eigen::Vector3f(voxel_size[0], voxel_size[1], voxel_size[2]);

	const bool valid =
		min_cloud_points__ >= 1 &&
		(crop_min__.head<3>().array() < crop_max__.head<3>().array()).all() &&
		(voxel_size__.array() > 0.0f).all() &&
		cluster_min_size__ >= 1 && cluster_max_size__ >= cluster_min_size__ &&
		pallet_width__ > 0.0 && pallet_height__ > 0.0 &&
		tol_width__ > 0.0 && tol_height__ > 0.0 &&
		tpl_cell_size__ > 0.0 && chi_threshold__ >= 0.0 && chi_threshold__ <= 1.0 &&
		pre_ransac_min_inliers__ >= 1 && pre_ransac_max_iter__ >= 1 && pre_ransac_distance_thresh__ > 0.0;

	if ( !valid )
	{
		ROS_ERROR("Invalid pallet_perceptor parameters.");
		return false;
	}
	return true;
}

// ---------------------------------------------------------------------------
// Service / subscription
// ---------------------------------------------------------------------------

bool PalletPerceptor::enableServiceCallback(
	target_detector::DetectorEnable::Request & __request,
	target_detector::DetectorEnable::Response & __response)
{
	if ( !enabled__ && __request.enable )
		subscribeToData();

	if ( enabled__ && !__request.enable )
	{
		unsubscribeFromData();

		target_detector::Detections msg;
		msg.header.stamp    = ros::Time::now();
		msg.header.frame_id = robot_frame__;
		msg.source_name     = source_name__;
		detections_publisher__.publish(msg);
	}

	enabled__ = __request.enable;
	__response.success = true;
	return true;
}

void PalletPerceptor::subscribeToData()
{
	point_cloud_subscriber__ = nh__.subscribe("point_cloud_in", 1, &PalletPerceptor::pointCloudCallback, this);
}

void PalletPerceptor::unsubscribeFromData()
{
	point_cloud_subscriber__.shutdown();
}

// ---------------------------------------------------------------------------
// Main callback
// ---------------------------------------------------------------------------

void PalletPerceptor::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & __cloud_in)
{
	if ( !enabled__ || !__cloud_in )
		return;

	if ( static_cast<int>(__cloud_in->width * __cloud_in->height) < min_cloud_points__ )
	{
		ROS_WARN_THROTTLE(2.0, "PalletPerceptor: too few points in input cloud");
		return;
	}

	if ( !saveSensorTransform(__cloud_in->header) )
		return;

	// 1) ROS -> PCL
	CloudT::Ptr cloud_in(new CloudT);
	pcl::fromROSMsg(*__cloud_in, *cloud_in);
	if ( cloud_in->empty() )
		return;

	// 2) ROI crop box (sensor optical frame)
	CloudT::Ptr cloud_crop(new CloudT);
	pcl::CropBox<PointT> crop;
	crop.setInputCloud(cloud_in);
	crop.setMin(crop_min__);
	crop.setMax(crop_max__);
	crop.filter(*cloud_crop);

	// 3) Voxel downsample
	CloudT::Ptr cloud_downsampled(new CloudT);
	pcl::VoxelGrid<PointT> vg;
	vg.setInputCloud(cloud_crop);
	vg.setLeafSize(voxel_size__.x(), voxel_size__.y(), voxel_size__.z());
	vg.filter(*cloud_downsampled);
	if ( cloud_downsampled->size() < kMinPostVoxelPoints )
		return;

	// 4) Y-band crop in sensor optical frame (gravity assumption: Y=down).
	//    Keep only points in the pallet height band above the floor.
	const float band_lo = static_cast<float>(floor_y__ - pallet_height__ - tol_height__);
	const float band_hi = static_cast<float>(floor_y__) - kFloorBandMargin;
	CloudT::Ptr cloud_band(new CloudT);
	cloud_band->reserve(cloud_downsampled->size());
	for ( const auto & pt : *cloud_downsampled )
		if ( pt.y >= band_lo && pt.y <= band_hi )
			cloud_band->push_back(pt);
	if ( cloud_band->size() < kMinPostBandPoints )
		return;

	// 5) Euclidean clustering
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud_band);
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(cluster_tolerance__);
	ec.setMinClusterSize(cluster_min_size__);
	ec.setMaxClusterSize(cluster_max_size__);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_band);
	std::vector<pcl::PointIndices> cluster_indices;
	ec.extract(cluster_indices);

	// 6) Try each cluster, keep the one with the highest chi score.
	bool found = false;
	double best_chi = -static_cast<double>(FLT_MAX);
	Eigen::Isometry3d T_best   = Eigen::Isometry3d::Identity();
	Eigen::Vector3f   dims_best(0.f, 0.f, 0.f);
	CloudT::Ptr       matched_best(new CloudT);

	for ( const auto & idx : cluster_indices )
	{
		CloudT::Ptr cluster(new CloudT);
		cluster->reserve(idx.indices.size());
		for ( int i : idx.indices )
			cluster->push_back((*cloud_band)[i]);

		Eigen::Isometry3d T;
		Eigen::Vector3f   dims;
		CloudT::Ptr       matched(new CloudT);
		double            chi;
		if ( !tryDetectPalletInCluster(cluster, T, dims, matched, chi) )
			continue;

		if ( chi > best_chi )
		{
			best_chi      = chi;
			T_best        = T;
			dims_best     = dims;
			matched_best  = matched;
			found         = true;
		}
	}

	if ( !found )
	{
		ROS_WARN_STREAM_THROTTLE(2.0, "No pallet detected. clusters=" << cluster_indices.size());
		return;
	}

	// 7) Build pose in sensor frame -> transform to robot frame for the published message.
	geometry_msgs::Pose pose_in_sensor;
	const Eigen::Quaterniond q(T_best.linear());
	pose_in_sensor.position.x    = T_best.translation().x();
	pose_in_sensor.position.y    = T_best.translation().y();
	pose_in_sensor.position.z    = T_best.translation().z();
	pose_in_sensor.orientation.x = q.x();
	pose_in_sensor.orientation.y = q.y();
	pose_in_sensor.orientation.z = q.z();
	pose_in_sensor.orientation.w = q.w();

	geometry_msgs::Pose pose_in_robot;
	tf2::doTransform(pose_in_sensor, pose_in_robot, T_sensor_to_robot__[__cloud_in->header.frame_id]);

	target_detector::Detections detections_msg;
	detections_msg.header          = __cloud_in->header;
	detections_msg.header.frame_id = robot_frame__;
	detections_msg.source_name     = source_name__;
	detections_msg.detections.resize(1);
	auto & det                = detections_msg.detections[0];
	det.type                  = target_detector::Detection::PALLET;
	det.pose.pose             = pose_in_robot;
	det.pose.covariance[0]    = -1.0;
	det.intensity             = 0.0;
	det.radius                = 0.0;
	det.supports              = matched_best->size();
	detections_publisher__.publish(detections_msg);

	if ( vizbose__ )
	{
		// inlier cloud in sensor frame
		matched_best->header.frame_id = __cloud_in->header.frame_id;
		pcl_conversions::toPCL(__cloud_in->header.stamp, matched_best->header.stamp);
		point_cloud_publisher__.publish(matched_best);

		// markers in sensor frame
		target_detector::Detections viz_msg     = detections_msg;
		viz_msg.header                          = __cloud_in->header; // sensor frame
		viz_msg.detections[0].pose.pose         = pose_in_sensor;
		publishMarkers(viz_msg, dims_best);
	}
}

// ---------------------------------------------------------------------------
// Detection helpers (per cluster)
// ---------------------------------------------------------------------------

bool PalletPerceptor::tryDetectPalletInCluster(
	CloudT::ConstPtr __cluster,
	Eigen::Isometry3d & __T_object_in_sensor,
	Eigen::Vector3f & __dims,
	CloudT::Ptr __matched_pts,
	double & __chi) const
{
	if ( static_cast<int>(__cluster->size()) < kMinClusterSize )
		return false;

	// Estimate pallet-face projection axes (sensor optical frame).
	const Eigen::Vector3f axis_u = estimateFaceNormalAxisU(__cluster);
	const Eigen::Vector3f axis_v(0.0f, 1.0f, 0.0f); // camera Y, always correct for upright pallet

	// Project + slide template
	std::vector<uint8_t> grid;
	int gcols = 0, grows = 0;
	float u_min = 0.f, v_min = 0.f;
	projectToFaceGrid(__cluster, axis_u, axis_v, grid, gcols, grows, u_min, v_min);
	if ( gcols < template_cols__ || grows < template_rows__ )
		return false;

	int best_co = 0, best_ro = 0;
	__chi = slideTemplate(grid, gcols, grows, best_co, best_ro);
	if ( __chi < chi_threshold__ )
		return false;

	// Width gate on matched window
	const float cs         = static_cast<float>(tpl_cell_size__);
	const float match_x_lo = u_min + best_co * cs;
	const float match_x_hi = u_min + (best_co + template_cols__) * cs;
	const float match_y_lo = v_min + best_ro * cs;
	const float match_y_hi = v_min + (best_ro + template_rows__) * cs;
	const float matched_W  = match_x_hi - match_x_lo;
	if ( std::abs(matched_W - static_cast<float>(pallet_width__)) > static_cast<float>(tol_width__) )
		return false;

	// Extract points falling inside the matched window
	__matched_pts->clear();
	__matched_pts->reserve(__cluster->size());
	for ( const auto & pt : *__cluster )
	{
		const float u = pt.x * axis_u.x() + pt.z * axis_u.z(); // axis_u.y() == 0
		if ( u >= match_x_lo && u <= match_x_hi && pt.y >= match_y_lo && pt.y <= match_y_hi )
			__matched_pts->push_back(pt);
	}
	if ( static_cast<int>(__matched_pts->size()) < kMinMatchedPoints )
		return false;

	// Pose: yaw + (x, z) from RANSAC stringer-zone plane fit (PCA fallback inside)
	float face_pos_x = 0.f, face_pos_z = 0.f, yaw = 0.f;
	const bool ransac_ok = computePalletPose(
		__matched_pts, match_y_lo, match_y_hi, face_pos_x, face_pos_z, yaw);

	const float face_pos_y = 0.5f * (match_y_lo + match_y_hi);

	// Build pallet axes in sensor frame: pal_x = horizontal width, pal_z = approach (back into face).
	Eigen::Vector3f pal_x(std::cos(yaw), 0.f, std::sin(yaw));
	if ( pal_x.x() < 0.f ) pal_x = -pal_x;
	Eigen::Vector3f pal_z(-pal_x.z(), 0.f, pal_x.x());
	if ( pal_z.z() > 0.f ) pal_z = -pal_z;
	const Eigen::Vector3f pal_y = pal_z.cross(pal_x);

	__T_object_in_sensor = Eigen::Isometry3d::Identity();
	__T_object_in_sensor.linear().col(0) = pal_x.cast<double>();
	__T_object_in_sensor.linear().col(1) = pal_y.cast<double>();
	__T_object_in_sensor.linear().col(2) = pal_z.cast<double>();
	__T_object_in_sensor.translation() = Eigen::Vector3d(face_pos_x, face_pos_y, face_pos_z);

	__dims = Eigen::Vector3f(matched_W, match_y_hi - match_y_lo, kObbDepthForViz);

	ROS_INFO_STREAM("  pallet candidate: chi=" << __chi
		<< " matched_W=" << matched_W
		<< " yaw=" << yaw * 180.0f / static_cast<float>(M_PI) << "deg"
		<< " ransac=" << (ransac_ok ? "OK" : "FALLBACK"));
	return true;
}

Eigen::Vector3f PalletPerceptor::estimateFaceNormalAxisU(CloudT::ConstPtr __cluster) const
{
	const Eigen::Vector3f fallback(1.0f, 0.0f, 0.0f); // camera X

	pcl::SACSegmentation<PointT> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setAxis(Eigen::Vector3f(0.f, 0.f, 1.f));
	seg.setEpsAngle(kPreRansacEpsAngleRad);
	seg.setDistanceThreshold(static_cast<float>(pre_ransac_distance_thresh__));
	seg.setMaxIterations(pre_ransac_max_iter__);
	seg.setInputCloud(__cluster);

	pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	seg.segment(*inliers, *coeff);

	if ( static_cast<int>(inliers->indices.size()) < pre_ransac_min_inliers__ ||
		 coeff->values.size() != 4 )
		return fallback;

	Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
	if ( n.z() > 0.f ) n = -n;
	if ( std::abs(n.y()) >= kVerticalNormalThresh )
		return fallback;

	Eigen::Vector3f axis_u = Eigen::Vector3f(-n.z(), 0.f, n.x()).normalized();
	if ( axis_u.x() < 0.f ) axis_u = -axis_u;
	return axis_u;
}

bool PalletPerceptor::computePalletPose(
	CloudT::ConstPtr __matched_pts,
	float __match_y_lo,
	float __match_y_hi,
	float & __face_pos_x,
	float & __face_pos_z,
	float & __yaw) const
{
	// Default: PCA fallback values (yaw + centroid x/z)
	Eigen::Vector4f c4;
	pcl::compute3DCentroid(*__matched_pts, c4);
	__face_pos_x = c4.x();
	__face_pos_z = c4.z();

	float cxx = 0.f, cxz = 0.f, czz = 0.f;
	for ( const auto & pt : *__matched_pts )
	{
		const float dx = pt.x - c4.x(), dz = pt.z - c4.z();
		cxx += dx * dx; cxz += dx * dz; czz += dz * dz;
	}
	__yaw = 0.5f * std::atan2(2.f * cxz, cxx - czz);

	// Stringer zone (excludes top deck so a box on the pallet cannot bias the fit)
	const float stringer_y_lo = __match_y_lo + static_cast<float>(tpl_top_deck_height__);
	CloudT::Ptr stringer_pts(new CloudT);
	stringer_pts->reserve(__matched_pts->size());
	for ( const auto & pt : *__matched_pts )
		if ( pt.y >= stringer_y_lo && pt.y <= __match_y_hi )
			stringer_pts->push_back(pt);
	if ( stringer_pts->empty() )
		return false;

	// RANSAC vertical-plane fit
	pcl::SACSegmentation<PointT> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setAxis(Eigen::Vector3f(0.f, 0.f, 1.f));
	seg.setEpsAngle(kPoseRansacEpsAngleRad);
	seg.setDistanceThreshold(kPoseRansacDistThresh);
	seg.setMaxIterations(kPoseRansacMaxIter);
	seg.setInputCloud(stringer_pts);

	pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	seg.segment(*inliers, *coeff);

	if ( static_cast<int>(inliers->indices.size()) < kPoseRansacMinInliers ||
		 coeff->values.size() != 4 )
		return false;

	Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
	if ( n.z() > 0.f ) n = -n;
	if ( std::abs(n.y()) >= kVerticalNormalThresh )
		return false;

	__yaw = std::atan2(n.x(), -n.z());

	// Refine X via ux-midpoint of inliers (density-independent in face-width direction).
	Eigen::Vector3f pal_x(std::cos(__yaw), 0.f, std::sin(__yaw));
	if ( pal_x.x() < 0.f ) pal_x = -pal_x;

	float x_sum = 0.f, z_sum = 0.f;
	float ux_min = FLT_MAX, ux_max = -FLT_MAX;
	for ( int i : inliers->indices )
	{
		const auto & pt = (*stringer_pts)[i];
		x_sum += pt.x;
		z_sum += pt.z;
		const float ux = pt.x * pal_x.x() + pt.z * pal_x.z();
		ux_min = std::min(ux_min, ux);
		ux_max = std::max(ux_max, ux);
	}
	const int n_in = static_cast<int>(inliers->indices.size());
	const float ix    = x_sum / n_in;
	const float iz    = z_sum / n_in;
	const float ix_ux = ix * pal_x.x() + iz * pal_x.z();
	const float ux_mid = 0.5f * (ux_min + ux_max);

	__face_pos_x = ix + (ux_mid - ix_ux) * pal_x.x();
	__face_pos_z = iz;
	return true;
}

// ---------------------------------------------------------------------------
// Template construction & matching
// ---------------------------------------------------------------------------

void PalletPerceptor::buildPalletTemplate()
{
	const float cs = static_cast<float>(tpl_cell_size__);
	const float dh = static_cast<float>(tpl_top_deck_height__);
	const float sh = static_cast<float>(tpl_stringer_height__);
	const float sw = static_cast<float>(tpl_stringer_width__);
	const float pw = static_cast<float>(pallet_width__);

	template_cols__ = std::max(1, static_cast<int>(std::round(pw / cs)));
	template_rows__ = std::max(1, static_cast<int>(std::round((dh + sh) / cs)));
	template_grid__.assign(template_cols__ * template_rows__, 0);

	// Solid blocks on the pallet face: top deck (full width) + 3 stringers below.
	struct Block { float x0, y0, w, h; };
	const Block blocks[] = {
		{0.f,                 0.f, pw, dh},
		{0.f,                 dh,  sw, sh},
		{pw / 2.f - sw / 2.f, dh,  sw, sh},
		{pw - sw,             dh,  sw, sh},
	};

	for ( const auto & b : blocks )
	{
		const int c0 = static_cast<int>(std::floor(b.x0 / cs));
		const int c1 = std::min(template_cols__, c0 + static_cast<int>(std::round(b.w / cs)));
		const int r0 = static_cast<int>(std::floor(b.y0 / cs));
		const int r1 = std::min(template_rows__, r0 + static_cast<int>(std::round(b.h / cs)));
		for ( int r = r0; r < r1; ++r )
			for ( int c = c0; c < c1; ++c )
				template_grid__[r * template_cols__ + c] = 1;
	}

	const int ones = std::count(template_grid__.begin(), template_grid__.end(), uint8_t(1));
	template_mu__  = static_cast<double>(ones) / static_cast<double>(template_grid__.size());

	ROS_INFO_STREAM("Pallet template " << template_cols__ << "x" << template_rows__
		<< " (" << tpl_cell_size__ * 100.0 << " cm/cell, mu=" << template_mu__ << ")");
	for ( int r = 0; r < template_rows__; ++r )
	{
		std::string row;
		for ( int c = 0; c < template_cols__; ++c )
			row += template_grid__[r * template_cols__ + c] ? '#' : '.';
		ROS_INFO_STREAM("  row " << r << ": " << row);
	}
}

void PalletPerceptor::projectToFaceGrid(
	CloudT::ConstPtr __slice,
	const Eigen::Vector3f & __axis_u,
	const Eigen::Vector3f & __axis_v,
	std::vector<uint8_t> & __grid,
	int & __gcols,
	int & __grows,
	float & __u_min,
	float & __v_min) const
{
	std::vector<float> us, vs;
	us.reserve(__slice->size());
	vs.reserve(__slice->size());
	for ( const auto & pt : *__slice )
	{
		const Eigen::Vector3f d(pt.x, pt.y, pt.z);
		us.push_back(d.dot(__axis_u));
		vs.push_back(d.dot(__axis_v));
	}

	__u_min = *std::min_element(us.begin(), us.end());
	__v_min = *std::min_element(vs.begin(), vs.end());
	const float u_max = *std::max_element(us.begin(), us.end());
	const float v_max = *std::max_element(vs.begin(), vs.end());

	const float cs = static_cast<float>(tpl_cell_size__);
	__gcols = std::max(1, static_cast<int>(std::ceil((u_max - __u_min) / cs)));
	__grows = std::max(1, static_cast<int>(std::ceil((v_max - __v_min) / cs)));

	__grid.assign(__gcols * __grows, 0);
	for ( size_t i = 0; i < us.size(); ++i )
	{
		const int c = std::min(__gcols - 1, static_cast<int>((us[i] - __u_min) / cs));
		const int r = std::min(__grows - 1, static_cast<int>((vs[i] - __v_min) / cs));
		__grid[r * __gcols + c] = 1;
	}
}

double PalletPerceptor::slideTemplate(
	const std::vector<uint8_t> & __grid,
	int __gcols,
	int __grows,
	int & __best_co,
	int & __best_ro) const
{
	__best_co = 0;
	__best_ro = 0;
	double best_chi = -static_cast<double>(FLT_MAX);
	const int total = template_cols__ * template_rows__;

	for ( int ro = 0; ro <= __grows - template_rows__; ++ro )
	{
		for ( int co = 0; co <= __gcols - template_cols__; ++co )
		{
			int agree = 0;
			for ( int tr = 0; tr < template_rows__; ++tr )
			{
				const int g_base = (ro + tr) * __gcols + co;
				const int t_base = tr * template_cols__;
				for ( int tc = 0; tc < template_cols__; ++tc )
					if ( template_grid__[t_base + tc] == __grid[g_base + tc] )
						++agree;
			}
			const double theta = static_cast<double>(agree) / total;
			const double chi   = (theta - template_mu__) / (1.0 - template_mu__);
			if ( chi > best_chi )
			{
				best_chi  = chi;
				__best_co = co;
				__best_ro = ro;
			}
		}
	}
	return best_chi;
}

// ---------------------------------------------------------------------------
// Markers & TF
// ---------------------------------------------------------------------------

void PalletPerceptor::publishMarkers(
	const target_detector::Detections & __detections_msg,
	const Eigen::Vector3f & __dims)
{
	if ( __detections_msg.detections.empty() )
		return;

	const auto & p = __detections_msg.detections[0].pose.pose;

	// OBB cube aligned with the pallet axes
	visualization_msgs::Marker cube;
	cube.header   = __detections_msg.header;
	cube.ns       = perceptor_name__;
	cube.id       = 0;
	cube.type     = visualization_msgs::Marker::CUBE;
	cube.action   = visualization_msgs::Marker::ADD;
	cube.pose     = p;
	cube.scale.x  = __dims.x();
	cube.scale.y  = __dims.y();
	cube.scale.z  = __dims.z();
	cube.color.r  = 0.1f; cube.color.g = 0.9f; cube.color.b = 0.1f; cube.color.a = 0.4f;
	viz_markers_publisher__.publish(cube);

	// Approach arrow along -pal_z (pal_z is column 2 of the rotation matrix).
	const double qx = p.orientation.x, qy = p.orientation.y, qz = p.orientation.z, qw = p.orientation.w;
	const Eigen::Vector3f pal_z(
		2.0f * static_cast<float>(qx * qz + qy * qw),
		2.0f * static_cast<float>(qy * qz - qx * qw),
		1.0f - 2.0f * static_cast<float>(qx * qx + qy * qy));
	const Eigen::Quaternionf q_arrow = Eigen::Quaternionf::FromTwoVectors(
		Eigen::Vector3f::UnitX(), -pal_z);

	visualization_msgs::Marker arrow;
	arrow.header           = __detections_msg.header;
	arrow.ns               = perceptor_name__ + "_arrow";
	arrow.id               = 0;
	arrow.type             = visualization_msgs::Marker::ARROW;
	arrow.action           = visualization_msgs::Marker::ADD;
	arrow.pose.position    = p.position;
	arrow.pose.orientation.x = q_arrow.x();
	arrow.pose.orientation.y = q_arrow.y();
	arrow.pose.orientation.z = q_arrow.z();
	arrow.pose.orientation.w = q_arrow.w();
	arrow.scale.x = 1.0; arrow.scale.y = 0.03; arrow.scale.z = 0.03;
	arrow.color.r = 1.0; arrow.color.g = 1.0; arrow.color.b = 0.0; arrow.color.a = 0.8;
	viz_markers_publisher__.publish(arrow);
}

bool PalletPerceptor::saveSensorTransform(const std_msgs::Header & __header)
{
	if ( T_sensor_to_robot__.contains(__header.frame_id) )
		return true;

	try
	{
		T_sensor_to_robot__[__header.frame_id] =
			tf_buffer__.lookupTransform(robot_frame__, __header.frame_id, __header.stamp, ros::Duration(1.0));
	}
	catch ( tf2::TransformException & ex )
	{
		ROS_WARN_STREAM("TF lookup " << robot_frame__ << " <- " << __header.frame_id << ": " << ex.what());
		return false;
	}

	ROS_INFO_STREAM("Static transform updated. From " << robot_frame__ << " to " << __header.frame_id);
	return true;
}

} // namespace TargetDetector
