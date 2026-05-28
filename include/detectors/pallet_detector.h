#ifndef DETECTORS__PALLET_DETECTOR_H
#define DETECTORS__PALLET_DETECTOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "detectors/pcl_base_detector.h"

namespace Detectors
{

// Algorithm parameters. Filled by the perceptor from the ROS param server and
// handed to the detector via configure(). All units are SI (m, radians).
struct PalletDetectorConfig
{
	// Euclidean clustering on the preprocessed cloud
	double cluster_tolerance = 0.05;
	int    cluster_min_size  = 500;
	int    cluster_max_size  = 200000;

	// Expected EUR/EPAL pallet dimensions
	double pallet_width  = 0.80;
	double pallet_height = 0.145;
	double tol_width     = 0.10;
	double tol_height    = 0.08;

	// Pre-projection RANSAC (estimates face normal for the projection axes)
	int    pre_ransac_max_iter        = 200;
	double pre_ransac_distance_thresh = 0.04;
	int    pre_ransac_min_inliers     = 80;

	// Front-face template matching
	double tpl_cell_size       = 0.02;
	double chi_threshold       = 0.40;
	double tpl_stringer_width  = 0.10;
	double tpl_top_deck_height = 0.04;
	double tpl_stringer_height = 0.06;
};

class PalletDetector : public PclBaseDetector
{
	public:
		// Cluster-level rejection categories. Each call to
		// tryDetectPalletInCluster() classifies the cluster into exactly one
		// of these. Counts accumulate over the detector's lifetime; reset
		// via resetRejectionCounts().
		enum class RejectionReason : int
		{
			Success = 0,
			ClusterTooSmall,         // initial cluster below kMinClusterSize
			SorTooSmall,             // SOR-filtered cluster below kMinClusterSize
			PreRansacFailed,         // no valid face plane / vertical normal
			ProjectionTooSmall,      // projection grid smaller than template
			TemplateChiLow,          // best chi below cfg__.chi_threshold
			WidthGateFailed,         // matched width outside tol_width
			MatchedPointsLow,        // too few points in the matched window
			PoseRansacFailed,        // pose-RANSAC stringer plane fit failed
			YawSanityFailed,         // pre/pose yaw disagreement above threshold
			_Count
		};

	protected:
		using PointT = pcl::PointXYZ;
		using CloudT = pcl::PointCloud<PointT>;

		PalletDetectorConfig cfg__;

		// Template state (built once in configure())
		std::vector<uint8_t> template_grid__;
		int    template_cols__ = 0;
		int    template_rows__ = 0;
		double template_mu__   = 0.0;
		bool   template_built__ = false;

		// Cached debug outputs from the last successful detect() call
		CloudT::Ptr last_ransac_inliers__;
		Eigen::Isometry3d last_ransac_marker_pose__ = Eigen::Isometry3d::Identity();
		Eigen::Vector3f   last_ransac_marker_dims__ = Eigen::Vector3f::Zero();
		Eigen::Vector3f   last_obb_dims__           = Eigen::Vector3f::Zero();
		int               last_cluster_count__      = 0;

		// Per-cluster rejection counters. Mutable so tryDetectPalletInCluster
		// can remain const.
		mutable std::array<size_t, static_cast<size_t>(RejectionReason::_Count)>
			rejection_counts__ = {};

	public:
		PalletDetector();
		~PalletDetector();

		// Set algorithm parameters and (re)build the EUR/EPAL face template.
		void configure(const PalletDetectorConfig & __cfg);

		// PclBaseDetector overrides
		virtual bool init();

		// Detect the best pallet candidate in a pre-processed cloud
		// (band-cropped + voxel-downsampled by the caller).
		// - __param: unused (kept for the base signature; pallet has fixed size)
		// - __cloud_in: preprocessed cloud (sensor optical frame)
		// - __cloud_out: matched-window points of the best candidate
		// - __T_O_C: pose of the pallet in the sensor frame
		// - __confidence_level: chi score of the best candidate (in [0, 1])
		// Returns true on success. Debug data from the same call is exposed
		// via the getLast*() methods below.
		virtual bool detect(
			const double & __param,
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in,
			pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_out,
			Eigen::Isometry3d & __T_O_C,
			double & __confidence_level,
			const bool & __vizbose = false);

		// Debug accessors. Valid only immediately after detect() returns true.
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr getLastRansacInliers() const { return last_ransac_inliers__; }
		Eigen::Vector3f   getLastObbDims()          const { return last_obb_dims__; }
		Eigen::Isometry3d getLastRansacMarkerPose() const { return last_ransac_marker_pose__; }
		Eigen::Vector3f   getLastRansacMarkerDims() const { return last_ransac_marker_dims__; }

		// Number of clusters considered in the most recent detect() call.
		int getLastClusterCount() const { return last_cluster_count__; }

		// Cluster-level rejection counters (see RejectionReason).
		static const char * rejectionName(RejectionReason __r);
		size_t getRejectionCount(RejectionReason __r) const
		{
			return rejection_counts__[static_cast<size_t>(__r)];
		}
		size_t getTotalAttempts() const;
		void   resetRejectionCounts() const;

	protected:

		// Helper used inside tryDetectPalletInCluster() at each gate.
		void recordRejection(RejectionReason __r) const
		{
			++rejection_counts__[static_cast<size_t>(__r)];
		}

		// Per-cluster pipeline: pre-RANSAC -> projection -> template -> width gate ->
		// pose RANSAC. On success outputs the pose, OBB dims, matched-window cloud,
		// the RANSAC inlier cloud, and an OBB marker (pose + dims) for the inlier
		// AABB in pallet-local frame.
		bool tryDetectPalletInCluster(
			CloudT::ConstPtr __cluster,
			Eigen::Isometry3d & __T_object_in_sensor,
			Eigen::Vector3f & __dims,
			CloudT::Ptr __matched_pts,
			CloudT::Ptr __ransac_inliers,
			Eigen::Isometry3d & __ransac_marker_pose,
			Eigen::Vector3f & __ransac_marker_dims,
			double & __chi) const;

		// RANSAC vertical-plane fit on the cluster -> projection axis_u along the
		// pallet face width direction. On failure, sets __rejected = true so the
		// caller can drop the cluster.
		Eigen::Vector3f estimateFaceNormalAxisU(
			CloudT::ConstPtr __cluster,
			bool & __rejected) const;

		// Refine yaw and (x, z) from the matched-window points via RANSAC plane fit
		// on the stringer zone. Also writes the RANSAC inlier subset to __ransac_inliers_out.
		// Returns true on success; false if no vertical plane could be fit.
		bool computePalletPose(
			CloudT::ConstPtr __matched_pts,
			float __match_y_lo,
			float __match_y_hi,
			CloudT::Ptr __ransac_inliers_out,
			float & __face_pos_x,
			float & __face_pos_z,
			float & __yaw) const;

		// EUR/EPAL end-face binary template construction (top deck + 3 stringers).
		void buildPalletTemplate();

		// Rasterize a cluster slice onto a 2D occupancy grid in face coords (u, v).
		void projectToFaceGrid(
			CloudT::ConstPtr __slice,
			const Eigen::Vector3f & __axis_u,
			const Eigen::Vector3f & __axis_v,
			std::vector<uint8_t> & __grid,
			int & __gcols,
			int & __grows,
			float & __u_min,
			float & __v_min) const;

		// Slide the pre-built template, return the best chi score and matched window origin.
		double slideTemplate(
			const std::vector<uint8_t> & __grid,
			int __gcols,
			int __grows,
			int & __best_co,
			int & __best_ro) const;

}; // class PalletDetector

} // namespace Detectors

#endif
