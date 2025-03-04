#include "target_detector/detector_reflector.h"

namespace TargetDetector
{

DetectorReflector::DetectorReflector()
{
	//
}

DetectorReflector::~DetectorReflector()
{
	//
}

bool DetectorReflector::configure(const std::map<std::string, double> & __params)
{
	// set config parameters
	if (__params.count("min_reflector_intensity") != 0 )
		min_reflector_intensity__ = __params.at("min_reflector_intensity");
	else
		min_reflector_intensity__ = 50.;
	if (__params.count("target_size") != 0 )
		reflector_width__ = __params.at("target_size");
	else
		reflector_width__ = 0.06;
	if (__params.count("max_target_range") != 0 )
		max_reflector_range__ = __params.at("max_target_range");
	else
		max_reflector_range__ = 25;

	return true;
}

DetectorType DetectorReflector::type() const
{
	return REFLECTOR;
}

bool DetectorReflector::detect(
	const double & __angle_init,
	const double & __angle_end,
	const std::vector<float> & __ranges,
	const std::vector<float> & __intensities,
	const Eigen::Isometry2d & __T_platform_sensor,
	std::vector<double> & __detections)
{
	// check empty conditions
	if ( __ranges.empty() )
		return false;
	if ( __intensities.empty() )
		return false;
	if ( __ranges.size() != __intensities.size() )
		return false;

	// precomputes fixed part involving tangent
	double angle_increment = ( __angle_end - __angle_init ) / __ranges.size();
	double tan_res_x2 = 2.*std::tan(std::fabs(angle_increment)/2.);

	// for each positive hit, do clustering
	std::vector<Cluster> clusters;
	bool in_cluster;
	double clustering_distance;
	double azimuth;
	double corrected_range;
	Eigen::Vector2d point_in_lidar;
	for ( unsigned int ii = 0; ii < __intensities.size(); ii++ )
	{
		if ( 	( __intensities[ii] > min_reflector_intensity__ ) &&
				( __ranges[ii] < max_reflector_range__ ) )
		{
			// From polar to cartesian coordinates in sensor frame
			azimuth = __angle_init + ii*angle_increment;
			corrected_range = correctDistortion(__ranges[ii]);
			point_in_lidar.x() = corrected_range * std::cos( azimuth );
			point_in_lidar.y() = corrected_range * std::sin( azimuth );

			// computes clustering parameters according scan data.
			// +0.04 to take into account typical range noise
			clustering_distance = __ranges[ii]*tan_res_x2+0.04;

			// clustering
			in_cluster = false;
			for ( auto & cluster : clusters )
			{
				if ( cluster.belongsCentroid(point_in_lidar, clustering_distance) )
				{
					cluster.addPoint(point_in_lidar, __intensities[ii]);
					in_cluster = true;
					break;
				}
			}
			if ( !in_cluster )
			{
				clusters.push_back(Cluster(point_in_lidar, __intensities[ii]));
			}
		}
	}

	// Filter out clusters without enough supports
	double reflector_aperture;
	unsigned int min_support_points;
	for ( auto & cluster : clusters )
	{
		// compute min_support_points
		reflector_aperture = 2*std::atan2(reflector_width__/2.0 , cluster.range());
		min_support_points = std::floor(reflector_aperture / angle_increment );
		if (min_support_points < 3) min_support_points = 3;

		// Only publish clusters with enough supports
		if ( cluster.supports() >= min_support_points)
		{
			// transform to robot frame
			cluster.transform(__T_platform_sensor);
			__detections.push_back((double)cluster.supports());
			__detections.push_back(cluster.intensity());
			__detections.push_back(cluster.centroid().x());
			__detections.push_back(cluster.centroid().y());
			__detections.push_back(0.05); //cxx not yet computed
			__detections.push_back(0.05); //cyy not yet computed
		}
	}

	return true;
}

double DetectorReflector::correctDistortion(const double & __range)
{
	// apply simple distortion model for range in reflector points
	double delta;
	if ( __range < 2.0 )
		delta = -0.01*__range+0.02;
	else
		delta = 0.0;
	return  __range + delta;
}


} // end of namespace
