#include "detectors/detector_pcl_barrel.h"

namespace Detectors
{

DetectorPclBarrel::DetectorPclBarrel()
{
	//
}

DetectorPclBarrel::~DetectorPclBarrel()
{
	//
}

//bool DetectorPclBarrel::init(const std::map<std::string, double> & __params)
bool DetectorPclBarrel::init()
{
	// check if param radius is in the map
	/*if ( !__params.find("radius") )
	{
		std::cout << "DetectorPclBarrel::init(): required radius param not set. Exit." << std::endl;
		return false;
	}

	// get radius
	radius__ = __params["radius"];
	if ( radius__ < 0.1 ) || ( radius__ > 2.0 )
	{
		std::cout << "DetectorPclBarrel::init(): radius should be in [0.1 , 2.0] m. Exit." << std::endl;
		return false;
	}*/

	// all ok
    return true;
}

bool DetectorPclBarrel::detect(
	const double & __param,
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in,
	pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_out,
	Eigen::Isometry3d & __T_O_C, // object wrt the camera (Check if better Affine3d ??)
	double & __confidence_level,
	const bool & __vizbose)
{
	// basic checks
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

	// check if no inliers
    if ( inliers->indices.empty() ) return false;

    // extract cylinder points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(__cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*__cloud_out);

    // compute cylinder pose. Model coefficients: [0-2]: point on axis, [3-5]: axis direction
    Eigen::Vector3d axis_point(coefficients->values[0], 0, coefficients->values[2]); //point at camera height (y=0)
	Eigen::Vector3d x_axis = -axis_point; // x of the cylinder always pointing to the camera
	x_axis.normalize();
    //Eigen::Vector3d z_axis(	coefficients->values[3], coefficients->values[4], coefficients->values[5]);
	//if ( z_axis.y() > 0 ) z_axis = -z_axis; // forces z axis of the barrel always pointing to the -Y hemisphere of the camera (meaninig usually Z barrel pointing up)
	//std::cout << "z_axis: " << z_axis.transpose() << std::endl;
	//z_axis.normalize();
	Eigen::Vector3d z_axis(0,-1,0); // forces gravity constraint, assumes camera Y pointing down
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    __T_O_C.linear().block<3,1>(0,0) = x_axis;
    __T_O_C.linear().block<3,1>(0,1) = y_axis;
    __T_O_C.linear().block<3,1>(0,2) = z_axis;
    __T_O_C.translation() = axis_point;

	// debugging
	//std::cout << std::endl << __T_O_C.matrix() << std::endl;

	__confidence_level =  1.0; // not used yet
    return true;
}

} // end of namespace
