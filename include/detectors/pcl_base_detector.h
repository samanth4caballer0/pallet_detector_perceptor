#ifndef DETECTORS__PCL_BASE_DETECTOR_H
#define DETECTORS__PCL_BASE_DETECTOR_H

//std C++
#include <iostream>
#include <algorithm>
#include <vector>
#include <map>

//Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//PCL (to do, remove those innecessary)
#include <pcl/point_types.h>
#include <pcl/common/transforms.h> //transforms
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h> // point indices for clustering
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h> //clustering
#include <pcl/kdtree/kdtree.h> // kd tree for clustering
#include <pcl/io/pcd_io.h>
#include <pcl/common/pca.h>
#include <pcl/filters/voxel_grid.h>

// namespace
namespace Detectors
{

// Base class for all detectors
class PclBaseDetector
{
	public:
        // Constructor
        PclBaseDetector();

        // Destructor
		~PclBaseDetector();

		// initializations
		//virtual bool init(const std::map<std::string, double> & __params) = 0;
		virtual bool init() = 0;

		// Detects a pallet from the point cloud
        virtual bool detect(
			const double & __param,
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in,
			pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_out,
			Eigen::Isometry3d & __T_O_C, // object wrt the camera (Check if better Affine3d ??)
			double & __confidence_level,
			const bool & __vizbose=false) = 0;

		// Crops the __cloud_in according min and max limits, provided as homogeneous points
		virtual void crop(
			const Eigen::Vector4f & __max_values,
			const Eigen::Vector4f & __min_values,
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in,
			pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_out,
		 	bool __negative = false);

		// downsample cloud with voxelization
		void voxelDownsampling(
			const Eigen::Vector3f & __voxel_size,
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in,
			pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_out);

		// Applies statistocal outlier removal to clean a cloud
		virtual void removeOutliers(
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in,
			pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_out);

		// Computes normals for point-typed __cloud_in and produces a normal-typed __cloud_out
		virtual void computeNormals(
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in,
			pcl::PointCloud<pcl::Normal>::Ptr __cloud_out);

		// flatten input cloud onto horizontal plane (assumed ZX), and sets output at XY plane
		virtual void projectToXYPlane(
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in,
			pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_out);

		// Returns the data vector index of maximum correlation between __data and __kernel
		virtual unsigned int correlation(
			const std::vector<double> & __data,
			const std::vector<double> & __kernel);

		// Save a cloud on disk, format .pcd
		virtual bool saveOnDisk(
			const std::string & __file_name,
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in);

		// Loads a .pcd file from disk
		virtual bool loadFromDisk(
			const std::string & __file_name,
			pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_out);


}; // end of class

} //end of namespace

#endif
