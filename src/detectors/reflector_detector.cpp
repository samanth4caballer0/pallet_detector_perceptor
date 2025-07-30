#include "detectors/reflector_detector.h"

namespace Detectors
{

bool ReflectorDetector::configure(const double & __reflector_size, const double & __min_reflector_intensity, const double & __max_detection_range)
{
	reflector_width__ = __reflector_size;
	min_reflector_intensity__ = __min_reflector_intensity;
	max_detection_range__ = __max_detection_range;
	return true;
}

std::vector<ReflectorDetection> ReflectorDetector::detect(const double & __angle_init, const double & __angle_end,
	const std::vector<float> & __ranges, const std::vector<float> & __intensities, const Eigen::Isometry2d & __T_platform_sensor)
{
	std::vector<ReflectorDetection> detections;

	// check empty conditions
	if ( __ranges.empty() || __intensities.empty() || (__ranges.size() != __intensities.size()) )
		return detections;

	// precomputes fixed part involving tangent
	double angle_increment = ( __angle_end - __angle_init ) / __ranges.size();
	double angular_spacing_factor = 2.0*std::tan(std::abs(angle_increment)/2.0);

	// for each positive hit, do clustering
	std::vector<Cluster> clusters;
	bool in_cluster;
	double clustering_distance;
	double azimuth;
	double corrected_range;
	Eigen::Vector2d point_in_lidar;
	for ( unsigned int i = 0; i < __intensities.size(); i++ )
	{
		if ( ( __intensities[i] > min_reflector_intensity__ ) && ( __ranges[i] < max_detection_range__ ) )
		{
			// From polar to cartesian coordinates in sensor frame
			azimuth = __angle_init + i*angle_increment;
			corrected_range = correctDistortion(__ranges[i]);
			point_in_lidar.x() = corrected_range * std::cos( azimuth );
			point_in_lidar.y() = corrected_range * std::sin( azimuth );

			// computes clustering parameters according scan data.
			// +0.04 to take into account typical range noise
			clustering_distance = __ranges[i] * angular_spacing_factor + 0.04;

			// clustering
			in_cluster = false;
			for ( auto & cluster : clusters )
			{
				if ( cluster.belongsCentroid(point_in_lidar, clustering_distance) )
				{
					cluster.addPoint(point_in_lidar, __intensities[i]);
					in_cluster = true;
					break;
				}
			}
			if ( !in_cluster )
			{
				clusters.push_back( Cluster(point_in_lidar, __intensities[i]) );
			}
		}
	}

	// Filter out clusters without enough supports
	double reflector_aperture;
	unsigned int min_support_points;
	for ( auto & cluster : clusters )
	{
		// compute min_support_points
		reflector_aperture = 2*std::atan2(reflector_width__/2.0, cluster.range());
		min_support_points = std::floor(reflector_aperture / angle_increment );
		if ( min_support_points < 3 )
			min_support_points = 3;

		// Only publish clusters with enough supports
		if ( cluster.supports() >= min_support_points)
		{
			// transform to robot frame
			cluster.transform(__T_platform_sensor);
			detections.push_back(
				ReflectorDetection(static_cast<double>(cluster.supports()), cluster.intensity(), cluster.centroid().x(), cluster.centroid().y()) );
		}
	}

	return detections;
}

double ReflectorDetector::correctDistortion(const double & __range)
{
	// apply simple distortion model for range in reflector points
	double delta;
	if ( __range < 2.0 )
		delta = -0.01*__range+0.02;
	else
		delta = 0.0;
	return  __range + delta;
}

}