#include "detectors/vertical_cylinder_detector.h"
#include <cmath>

namespace Detectors
{

VerticalCylinderDetector::VerticalCylinderDetector()
{
	//
}

VerticalCylinderDetector::~VerticalCylinderDetector()
{
	//
}

//bool VerticalCylinderDetector::init(const std::map<std::string, double> & __params)
bool VerticalCylinderDetector::init()
{
	// check if param radius is in the map
	/*if ( !__params.find("radius") )
	{
		std::cout << "VerticalCylinderDetector::init(): required radius param not set. Exit." << std::endl;
		return false;
	}

	// get radius
	radius__ = __params["radius"];
	if ( radius__ < 0.1 ) || ( radius__ > 2.0 )
	{
		std::cout << "VerticalCylinderDetector::init(): radius should be in [0.1 , 2.0] m. Exit." << std::endl;
		return false;
	}*/

	// all ok
    return true;
}

bool VerticalCylinderDetector::detect(
	const double & __param,
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in,
	pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_out,
	Eigen::Isometry3d & __T_O_C, // object wrt the camera (Check if better Affine3d ??)
	double & __confidence_level,
	const bool & __vizbose)
{
	// basic checks
	if ( !std::isfinite(__param) || __param <= 0.0 ) return false;
    if ( __cloud_in->empty() ) return false;

    // estimate normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(__cloud_in);
    ne.setSearchMethod(tree);
    ne.setKSearch(50); // uses N neighbors to compute normals
    ne.compute(*normals);

    // Cylinder segmentation
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices); // indice of inlier points
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(false);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.04); // distance to the model to be an inlier
    seg.setRadiusLimits(__param*0.98, __param*1.02);
    seg.setInputCloud(__cloud_in);
    seg.setInputNormals(normals);
    seg.segment(*inliers, *coefficients);

	// check if enough inliers.
	if ( inliers->indices.size() < 200 ) return false;

    // extract cylinder points (inliers)
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(__cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*__cloud_out);

	//check AABB (axis aligned bounding box)
	pcl::PointXYZ min_pt, max_pt;
	pcl::getMinMax3D(*__cloud_out, min_pt, max_pt);
	double diameter = __param*2;
	double dx = max_pt.x - min_pt.x; // distance between min and max in the x dimension (horizontal)
	if ( ( dx < 0.8*diameter ) || ( dx > 1.1*diameter ) ) return false;
	//std::cout << "Min point: " << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << std::endl;
	//std::cout << "Max point: " << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << std::endl;

	// compute cylinder pose. Model coefficients: [0-2]: point on axis, [3-5]: axis direction
	Eigen::Vector3d target_point(coefficients->values[0], 0, coefficients->values[2]); //point at camera height (y=0)
  	Eigen::Vector3d target_x_axis = -target_point; // x of the cylinder always pointing to the camera
  	target_x_axis.normalize();
  	Eigen::Vector3d target_z_axis(0,-1,0); // forces gravity constraint, assumes camera Y pointing down
	Eigen::Vector3d target_y_axis = target_z_axis.cross(target_x_axis);
	__T_O_C = Eigen::Isometry3d::Identity();
	__T_O_C.linear().block<3,1>(0,0) = target_x_axis;
	__T_O_C.linear().block<3,1>(0,1) = target_y_axis;
	__T_O_C.linear().block<3,1>(0,2) = target_z_axis;
	__T_O_C.translation() = target_point;
  	//std::cout << std::endl << __T_O_C.matrix() << std::endl;

  	// check detection is convex. Project points to -target_x_axis, and keeps cos angle between points and -target_x_axis
  	double point_projection;
  	double point_cos_angle;
  	double range_central = 0;
  	double range_lateral = 0;
  	unsigned int count_central = 0;
  	unsigned int count_lateral = 0;
  	Eigen::Vector2d pt;
  	unsigned int ii;
  	for ( ii=0; ii<__cloud_out->size(); ii++)
  	{
  		pt << __cloud_out->points[ii].x, __cloud_out->points[ii].z;
  		point_projection = - target_x_axis.x()*pt(0) - target_x_axis.z()*pt(1);
  		point_cos_angle = point_projection / pt.norm();
  		if ( std::abs(point_cos_angle) > 0.99) // central barrel points
  		{
  			range_central += point_projection;
  			count_central ++;
  		}
  		else // lateral points
  		{
  			range_lateral += point_projection;
  			count_lateral ++;
  		}
  	}
  	if ( ( count_central == 0 ) || ( count_lateral == 0 ) )
  	{
  		std::cout << "count_central: " << count_central << "; count_lateral: " << count_lateral << std::endl;
  		return false;
  	}
  	range_central = range_central / count_central;
  	range_lateral = range_lateral / count_lateral;
  	//std::cout << "range_central: "<< range_central << std::endl;
  	//std::cout << "range_lateral: "<< range_lateral << std::endl;
  	if ( range_central > range_lateral )
  	{
		std::cout << "Not convex. Central points further than lateral points." << std::endl; 
  		return false;
  	}

	// set confidence level (not used yet)
  	__confidence_level =  1.0;

	// the end
	return true;
}

} // end of namespace
