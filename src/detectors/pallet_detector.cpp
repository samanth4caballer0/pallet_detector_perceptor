#include "detectors/pallet_detector.h"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <iostream>

namespace Detectors
{

namespace
{
	// ---- Algorithm constants (not exposed as user params) ----
	constexpr float kPreRansacEpsAngleRad   = 0.44f;  // ~25 deg
	constexpr float kPoseRansacEpsAngleRad  = 0.785f; // ~45 deg
	constexpr float kPoseRansacDistThresh   = 0.02f;
	constexpr int   kPoseRansacMaxIter      = 300;
	constexpr int   kPoseRansacMinInliers   = 60;
	constexpr float kVerticalNormalThresh   = 0.5f;
	constexpr int   kMinClusterSize         = 50;
	constexpr int   kMinMatchedPoints       = 30;
	constexpr int   kSorMeanK               = 15;
	constexpr float kSorStddevThresh        = 1.5f;
	constexpr float kMaxPrePoseYawDiffRad   = 0.14f;  // ~8 deg
	constexpr float kObbDepthForViz         = 0.05f;
}

// ---------------------------------------------------------------------------
// Construction / configuration
// ---------------------------------------------------------------------------

PalletDetector::PalletDetector()
{
	last_ransac_inliers__.reset(new CloudT);
}

PalletDetector::~PalletDetector()
{
}

void PalletDetector::configure(const PalletDetectorConfig & __cfg)
{
	cfg__ = __cfg;
	buildPalletTemplate();
}

bool PalletDetector::init()
{
	return template_built__;
}

// ---------------------------------------------------------------------------
// Main entry point
// ---------------------------------------------------------------------------

bool PalletDetector::detect(
	const double & /*__param*/,
	CloudT::ConstPtr __cloud_in,
	CloudT::Ptr __cloud_out,
	Eigen::Isometry3d & __T_O_C,
	double & __confidence_level,
	const bool & /*__vizbose*/)
{
	last_cluster_count__ = 0;
	if (!template_built__ || !__cloud_in || __cloud_in->empty())
		return false;

	// 1) Euclidean clustering
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(__cloud_in);
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(cfg__.cluster_tolerance);
	ec.setMinClusterSize(cfg__.cluster_min_size);
	ec.setMaxClusterSize(cfg__.cluster_max_size);
	ec.setSearchMethod(tree);
	ec.setInputCloud(__cloud_in);
	std::vector<pcl::PointIndices> cluster_indices;
	ec.extract(cluster_indices);
	last_cluster_count__ = static_cast<int>(cluster_indices.size());

	// 2) Try each cluster, keep the one with the highest chi score.
	bool found = false;
	double best_chi = -static_cast<double>(FLT_MAX);
	Eigen::Isometry3d T_best                  = Eigen::Isometry3d::Identity();
	Eigen::Vector3f   dims_best               = Eigen::Vector3f::Zero();
	CloudT::Ptr       matched_best(new CloudT);
	CloudT::Ptr       ransac_inliers_best(new CloudT);
	Eigen::Isometry3d ransac_marker_pose_best = Eigen::Isometry3d::Identity();
	Eigen::Vector3f   ransac_marker_dims_best = Eigen::Vector3f::Zero();

	for (const auto & idx : cluster_indices)
	{
		CloudT::Ptr cluster(new CloudT);
		cluster->reserve(idx.indices.size());
		for (int i : idx.indices)
			cluster->push_back((*__cloud_in)[i]);

		Eigen::Isometry3d T;
		Eigen::Vector3f   dims;
		CloudT::Ptr       matched(new CloudT);
		CloudT::Ptr       inliers(new CloudT);
		Eigen::Isometry3d ransac_marker_pose;
		Eigen::Vector3f   ransac_marker_dims;
		double            chi;

		if (!tryDetectPalletInCluster(cluster, T, dims, matched, inliers,
		                              ransac_marker_pose, ransac_marker_dims, chi))
			continue;

		if (chi > best_chi)
		{
			best_chi                = chi;
			T_best                  = T;
			dims_best               = dims;
			matched_best            = matched;
			ransac_inliers_best     = inliers;
			ransac_marker_pose_best = ransac_marker_pose;
			ransac_marker_dims_best = ransac_marker_dims;
			found                   = true;
		}
	}

	if (!found)
		return false;

	// 3) Populate outputs
	*__cloud_out          = *matched_best;
	__T_O_C               = T_best;
	__confidence_level    = best_chi;

	// 4) Cache debug data for the perceptor to retrieve via getLast*()
	last_obb_dims__              = dims_best;
	last_ransac_inliers__        = ransac_inliers_best;
	last_ransac_marker_pose__    = ransac_marker_pose_best;
	last_ransac_marker_dims__    = ransac_marker_dims_best;

	return true;
}

// ---------------------------------------------------------------------------
// Per-cluster pipeline
// ---------------------------------------------------------------------------

bool PalletDetector::tryDetectPalletInCluster(
	CloudT::ConstPtr __cluster,
	Eigen::Isometry3d & __T_object_in_sensor,
	Eigen::Vector3f & __dims,
	CloudT::Ptr __matched_pts,
	CloudT::Ptr __ransac_inliers,
	Eigen::Isometry3d & __ransac_marker_pose,
	Eigen::Vector3f & __ransac_marker_dims,
	double & __chi) const
{
	if (static_cast<int>(__cluster->size()) < kMinClusterSize)
		return false;

	// Statistical outlier removal: drop points whose mean distance to their
	// K nearest neighbours is more than kSorStddevThresh*sigma above the
	// cluster average. Removes scattered ray-direction noise that pulls the
	// pose RANSAC plane fit off-axis without losing real face points.
	CloudT::Ptr cluster_clean(new CloudT);
	{
		pcl::StatisticalOutlierRemoval<PointT> sor;
		sor.setInputCloud(__cluster);
		sor.setMeanK(kSorMeanK);
		sor.setStddevMulThresh(kSorStddevThresh);
		sor.filter(*cluster_clean);
	}
	if (static_cast<int>(cluster_clean->size()) < kMinClusterSize)
		return false;

	// Estimate pallet-face projection axes. If pre-RANSAC can't confidently
	// identify the face plane (±25 deg of camera Z), reject the cluster outright.
	bool pre_ransac_rejected = false;
	const Eigen::Vector3f axis_u = estimateFaceNormalAxisU(cluster_clean, pre_ransac_rejected);
	if (pre_ransac_rejected)
		return false;
	const Eigen::Vector3f axis_v(0.0f, 1.0f, 0.0f); // camera Y, always correct for upright pallet

	// Project + slide template
	std::vector<uint8_t> grid;
	int   gcols = 0, grows = 0;
	float u_min = 0.f, v_min = 0.f;
	projectToFaceGrid(cluster_clean, axis_u, axis_v, grid, gcols, grows, u_min, v_min);
	if (gcols < template_cols__ || grows < template_rows__)
		return false;

	int best_co = 0, best_ro = 0;
	__chi = slideTemplate(grid, gcols, grows, best_co, best_ro);
	if (__chi < cfg__.chi_threshold)
		return false;

	// Width gate on matched window
	const float cs         = static_cast<float>(cfg__.tpl_cell_size);
	const float match_x_lo = u_min + best_co * cs;
	const float match_x_hi = u_min + (best_co + template_cols__) * cs;
	const float match_y_lo = v_min + best_ro * cs;
	const float match_y_hi = v_min + (best_ro + template_rows__) * cs;
	const float matched_W  = match_x_hi - match_x_lo;
	if (std::abs(matched_W - static_cast<float>(cfg__.pallet_width)) > static_cast<float>(cfg__.tol_width))
		return false;

	// Extract points falling inside the matched window
	__matched_pts->clear();
	__matched_pts->reserve(cluster_clean->size());
	for (const auto & pt : *cluster_clean)
	{
		const float u = pt.x * axis_u.x() + pt.z * axis_u.z(); // axis_u.y() == 0
		if (u >= match_x_lo && u <= match_x_hi && pt.y >= match_y_lo && pt.y <= match_y_hi)
			__matched_pts->push_back(pt);
	}
	if (static_cast<int>(__matched_pts->size()) < kMinMatchedPoints)
		return false;

	// Pose: yaw + (x, z) from RANSAC stringer-zone plane fit. No PCA fallback.
	float face_pos_x = 0.f, face_pos_z = 0.f, yaw = 0.f;
	if (!computePalletPose(__matched_pts, match_y_lo, match_y_hi,
	                       __ransac_inliers, face_pos_x, face_pos_z, yaw))
		return false;

	// Sanity: pose-RANSAC's yaw should agree with the pre-RANSAC's implicit yaw
	// (the angle of axis_u). If they disagree by more than ~8 deg, the two
	// stages are not seeing the same surface — reject the cluster.
	const float pre_yaw  = std::atan2(axis_u.z(), axis_u.x());
	const float yaw_diff = std::remainder(yaw - pre_yaw, 2.0f * static_cast<float>(M_PI));
	if (std::abs(yaw_diff) > kMaxPrePoseYawDiffRad)
		return false;

	const float face_pos_y = 0.5f * (match_y_lo + match_y_hi);

	// Build pallet axes in sensor frame: pal_x = horizontal width, pal_z = approach (back into face).
	Eigen::Vector3f pal_x(std::cos(yaw), 0.f, std::sin(yaw));
	if (pal_x.x() < 0.f) pal_x = -pal_x;
	Eigen::Vector3f pal_z(-pal_x.z(), 0.f, pal_x.x());
	if (pal_z.z() > 0.f) pal_z = -pal_z;
	const Eigen::Vector3f pal_y = pal_z.cross(pal_x);

	// Published object frame follows the navigator/tracker ALVAR convention:
	//   x = forward  (face normal, pointing back toward the camera) -> pal_z
	//   y = lateral  (along the pallet face width)                  -> pal_x
	//   z = up       (vertical)                                     -> pal_y
	__T_object_in_sensor = Eigen::Isometry3d::Identity();
	// Previous convention (x=width, y=up, z=approach) — replaced 2026-05-22:
	// __T_object_in_sensor.linear().col(0) = pal_x.cast<double>();
	// __T_object_in_sensor.linear().col(1) = pal_y.cast<double>();
	// __T_object_in_sensor.linear().col(2) = pal_z.cast<double>();
	__T_object_in_sensor.linear().col(0) = pal_z.cast<double>();
	__T_object_in_sensor.linear().col(1) = pal_x.cast<double>();
	__T_object_in_sensor.linear().col(2) = pal_y.cast<double>();
	__T_object_in_sensor.translation() = Eigen::Vector3d(face_pos_x, face_pos_y, face_pos_z);

	// dims ordered to match the object axes above: (depth, width, height)
	// __dims = Eigen::Vector3f(matched_W, match_y_hi - match_y_lo, kObbDepthForViz);
	__dims = Eigen::Vector3f(kObbDepthForViz, matched_W, match_y_hi - match_y_lo);

	// RANSAC-inlier AABB in pallet-local frame, used to size the debug OBB marker.
	float lx_min = FLT_MAX, lx_max = -FLT_MAX;
	float ly_min = FLT_MAX, ly_max = -FLT_MAX;
	float lz_min = FLT_MAX, lz_max = -FLT_MAX;
	for (const auto & pt : *__ransac_inliers)
	{
		const Eigen::Vector3f rel(pt.x - face_pos_x, pt.y - face_pos_y, pt.z - face_pos_z);
		const float lx = rel.dot(pal_x);
		const float ly = rel.dot(pal_y);
		const float lz = rel.dot(pal_z);
		lx_min = std::min(lx_min, lx); lx_max = std::max(lx_max, lx);
		ly_min = std::min(ly_min, ly); ly_max = std::max(ly_max, ly);
		lz_min = std::min(lz_min, lz); lz_max = std::max(lz_max, lz);
	}
	const float cx_local = 0.5f * (lx_min + lx_max);
	const float cy_local = 0.5f * (ly_min + ly_max);
	const float cz_local = 0.5f * (lz_min + lz_max);
	const Eigen::Vector3f center_sensor =
		Eigen::Vector3f(face_pos_x, face_pos_y, face_pos_z)
		+ cx_local * pal_x + cy_local * pal_y + cz_local * pal_z;

	__ransac_marker_pose = __T_object_in_sensor; // same orientation as the main OBB
	__ransac_marker_pose.translation() = center_sensor.cast<double>();
	// ordered to match the object axes: (depth=pal_z, width=pal_x, height=pal_y)
	// __ransac_marker_dims = Eigen::Vector3f(
	// 	lx_max - lx_min,
	// 	ly_max - ly_min,
	// 	std::max(lz_max - lz_min, 0.005f)); // floor at 5 mm so it's visible in RViz
	__ransac_marker_dims = Eigen::Vector3f(
		std::max(lz_max - lz_min, 0.005f), // depth  (x): floor at 5 mm so it's visible in RViz
		lx_max - lx_min,                   // width  (y)
		ly_max - ly_min);                  // height (z)

	std::cout << "  pallet candidate: chi=" << __chi
	          << " matched_W=" << matched_W
	          << " yaw=" << yaw * 180.0f / static_cast<float>(M_PI) << " deg"
	          << " inliers=" << __ransac_inliers->size()
	          << std::endl;
	return true;
}

Eigen::Vector3f PalletDetector::estimateFaceNormalAxisU(
	CloudT::ConstPtr __cluster,
	bool & __rejected) const
{
	__rejected = false;

	pcl::SACSegmentation<PointT> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setAxis(Eigen::Vector3f(0.f, 0.f, 1.f));
	seg.setEpsAngle(kPreRansacEpsAngleRad);
	seg.setDistanceThreshold(static_cast<float>(cfg__.pre_ransac_distance_thresh));
	seg.setMaxIterations(cfg__.pre_ransac_max_iter);
	seg.setInputCloud(__cluster);

	pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr      inliers(new pcl::PointIndices);
	seg.segment(*inliers, *coeff);

	if (static_cast<int>(inliers->indices.size()) < cfg__.pre_ransac_min_inliers ||
		coeff->values.size() != 4)
	{
		__rejected = true;
		return Eigen::Vector3f::UnitX(); // unused by caller, kept for type
	}

	Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
	if (n.z() > 0.f) n = -n;
	if (std::abs(n.y()) >= kVerticalNormalThresh)
	{
		__rejected = true;
		return Eigen::Vector3f::UnitX();
	}

	Eigen::Vector3f axis_u = Eigen::Vector3f(-n.z(), 0.f, n.x()).normalized();
	if (axis_u.x() < 0.f) axis_u = -axis_u;
	return axis_u;
}

bool PalletDetector::computePalletPose(
	CloudT::ConstPtr __matched_pts,
	float __match_y_lo,
	float __match_y_hi,
	CloudT::Ptr __ransac_inliers_out,
	float & __face_pos_x,
	float & __face_pos_z,
	float & __yaw) const
{
	// Stringer zone (excludes top deck so a box on the pallet cannot bias the fit)
	const float stringer_y_lo = __match_y_lo + static_cast<float>(cfg__.tpl_top_deck_height);
	CloudT::Ptr stringer_pts(new CloudT);
	stringer_pts->reserve(__matched_pts->size());
	for (const auto & pt : *__matched_pts)
		if (pt.y >= stringer_y_lo && pt.y <= __match_y_hi)
			stringer_pts->push_back(pt);
	if (stringer_pts->empty())
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
	pcl::PointIndices::Ptr      inliers(new pcl::PointIndices);
	seg.segment(*inliers, *coeff);

	if (static_cast<int>(inliers->indices.size()) < kPoseRansacMinInliers ||
		coeff->values.size() != 4)
		return false;

	Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
	if (n.z() > 0.f) n = -n;
	if (std::abs(n.y()) >= kVerticalNormalThresh)
		return false;

	__yaw = std::atan2(n.x(), -n.z());

	// Refine X via ux-midpoint of inliers (density-independent in face-width direction).
	Eigen::Vector3f pal_x(std::cos(__yaw), 0.f, std::sin(__yaw));
	if (pal_x.x() < 0.f) pal_x = -pal_x;

	float x_sum = 0.f, z_sum = 0.f;
	float ux_min = FLT_MAX, ux_max = -FLT_MAX;

	__ransac_inliers_out->clear();
	__ransac_inliers_out->reserve(inliers->indices.size());

	for (int i : inliers->indices)
	{
		const auto & pt = (*stringer_pts)[i];
		__ransac_inliers_out->push_back(pt);
		x_sum += pt.x;
		z_sum += pt.z;
		const float ux = pt.x * pal_x.x() + pt.z * pal_x.z();
		ux_min = std::min(ux_min, ux);
		ux_max = std::max(ux_max, ux);
	}
	const int   n_in   = static_cast<int>(inliers->indices.size());
	const float ix     = x_sum / n_in;
	const float iz     = z_sum / n_in;
	const float ix_ux  = ix * pal_x.x() + iz * pal_x.z();
	const float ux_mid = 0.5f * (ux_min + ux_max);

	__face_pos_x = ix + (ux_mid - ix_ux) * pal_x.x();
	__face_pos_z = iz;
	return true;
}

// ---------------------------------------------------------------------------
// Template construction & matching
// ---------------------------------------------------------------------------

void PalletDetector::buildPalletTemplate()
{
	const float cs = static_cast<float>(cfg__.tpl_cell_size);
	const float dh = static_cast<float>(cfg__.tpl_top_deck_height);
	const float sh = static_cast<float>(cfg__.tpl_stringer_height);
	const float sw = static_cast<float>(cfg__.tpl_stringer_width);
	const float pw = static_cast<float>(cfg__.pallet_width);

	template_cols__ = std::max(1, static_cast<int>(std::round(pw / cs)));
	template_rows__ = std::max(1, static_cast<int>(std::round((dh + sh) / cs)));
	template_grid__.assign(template_cols__ * template_rows__, 0);

	struct Block { float x0, y0, w, h; };
	const Block blocks[] = {
		{0.f,                 0.f, pw, dh},
		{0.f,                 dh,  sw, sh},
		{pw / 2.f - sw / 2.f, dh,  sw, sh},
		{pw - sw,             dh,  sw, sh},
	};

	for (const auto & b : blocks)
	{
		const int c0 = static_cast<int>(std::floor(b.x0 / cs));
		const int c1 = std::min(template_cols__, c0 + static_cast<int>(std::round(b.w / cs)));
		const int r0 = static_cast<int>(std::floor(b.y0 / cs));
		const int r1 = std::min(template_rows__, r0 + static_cast<int>(std::round(b.h / cs)));
		for (int r = r0; r < r1; ++r)
			for (int c = c0; c < c1; ++c)
				template_grid__[r * template_cols__ + c] = 1;
	}

	const int ones = std::count(template_grid__.begin(), template_grid__.end(), uint8_t(1));
	template_mu__  = static_cast<double>(ones) / static_cast<double>(template_grid__.size());

	std::cout << "Pallet template " << template_cols__ << "x" << template_rows__
	          << " (" << cfg__.tpl_cell_size * 100.0 << " cm/cell, mu=" << template_mu__ << ")"
	          << std::endl;
	for (int r = 0; r < template_rows__; ++r)
	{
		std::string row;
		for (int c = 0; c < template_cols__; ++c)
			row += template_grid__[r * template_cols__ + c] ? '#' : '.';
		std::cout << "  row " << r << ": " << row << std::endl;
	}

	template_built__ = true;
}

void PalletDetector::projectToFaceGrid(
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
	for (const auto & pt : *__slice)
	{
		const Eigen::Vector3f d(pt.x, pt.y, pt.z);
		us.push_back(d.dot(__axis_u));
		vs.push_back(d.dot(__axis_v));
	}

	__u_min = *std::min_element(us.begin(), us.end());
	__v_min = *std::min_element(vs.begin(), vs.end());
	const float u_max = *std::max_element(us.begin(), us.end());
	const float v_max = *std::max_element(vs.begin(), vs.end());

	const float cs = static_cast<float>(cfg__.tpl_cell_size);
	__gcols = std::max(1, static_cast<int>(std::ceil((u_max - __u_min) / cs)));
	__grows = std::max(1, static_cast<int>(std::ceil((v_max - __v_min) / cs)));

	__grid.assign(__gcols * __grows, 0);
	for (size_t i = 0; i < us.size(); ++i)
	{
		const int c = std::min(__gcols - 1, static_cast<int>((us[i] - __u_min) / cs));
		const int r = std::min(__grows - 1, static_cast<int>((vs[i] - __v_min) / cs));
		__grid[r * __gcols + c] = 1;
	}
}

double PalletDetector::slideTemplate(
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

	for (int ro = 0; ro <= __grows - template_rows__; ++ro)
	{
		for (int co = 0; co <= __gcols - template_cols__; ++co)
		{
			int agree = 0;
			for (int tr = 0; tr < template_rows__; ++tr)
			{
				const int g_base = (ro + tr) * __gcols + co;
				const int t_base = tr * template_cols__;
				for (int tc = 0; tc < template_cols__; ++tc)
					if (template_grid__[t_base + tc] == __grid[g_base + tc])
						++agree;
			}
			const double theta = static_cast<double>(agree) / total;
			const double chi   = (theta - template_mu__) / (1.0 - template_mu__);
			if (chi > best_chi)
			{
				best_chi  = chi;
				__best_co = co;
				__best_ro = ro;
			}
		}
	}
	return best_chi;
}

} // namespace Detectors
