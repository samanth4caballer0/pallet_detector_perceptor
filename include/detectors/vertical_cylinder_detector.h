#ifndef DETECTORS__VERTICAL_CYLINDER_DETECTOR_H
#define DETECTORS__VERTICAL_CYLINDER_DETECTOR_H

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// eigen
#include <Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// this library
#include "detectors/pcl_base_detector.h"

namespace Detectors
{

class VerticalCylinderDetector : public PclBaseDetector
{
	protected:
		//double param__;

	public:

		// Constructor
	    VerticalCylinderDetector();

		// Destructor
	    ~VerticalCylinderDetector();

	    // Initialize detector with known cylinder radius
	    //virtual bool init(const std::map<std::string, double> & __params);
		virtual bool init();

		// Detect vertical cylinder and output inlier points + pose
		virtual bool detect(
			const double & __param,
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in,
			pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_out,
			Eigen::Isometry3d & __T_O_C, // object wrt the camera
			double & __confidence_level,
			const bool & __vizbose=false);

}; // end of class

} // end of namespace

#endif
