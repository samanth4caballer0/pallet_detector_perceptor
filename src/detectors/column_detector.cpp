#include "detectors/column_detector.h"

#include <cmath>

namespace Detectors
{

bool ColumnDetector::configure(
	const double & __column_size,
	const double & __max_detection_range,
	const int & __override_support_points)
{
	if ( !std::isfinite(__column_size) || __column_size <= 0.0 )
		return false;
	if ( !std::isfinite(__max_detection_range) || __max_detection_range <= 0.0 )
		return false;
	if ( __override_support_points < 0 )
		return false;

	column_size__ = __column_size;
	max_detection_range__ = __max_detection_range;
	override_support_points__ = __override_support_points;
	return true;
}

std::vector<ColumnDetection> ColumnDetector::detect(
	const double & __angle_init,
	const double & __angle_end,
	const std::vector<float> & __ranges)
{
	std::vector<ColumnDetection> detections;

	if ( __ranges.empty() )
		return detections;

	const double angle_increment = (__angle_end - __angle_init) / static_cast<double>(__ranges.size());
	const double angle_increment_abs = std::abs(angle_increment);
	if ( !std::isfinite(angle_increment_abs) || angle_increment_abs <= 0.0 )
		return detections;

	const double angular_spacing_factor = 2.0 * std::tan(angle_increment_abs / 2.0);

	std::vector<Cluster> clusters;
	for ( size_t ii = 0; ii < __ranges.size(); ++ii )
	{
		const double range = __ranges[ii];
		if ( !std::isfinite(range) || range <= 0.0 || range >= max_detection_range__ )
			continue;

		const double azimuth = __angle_init + static_cast<double>(ii) * angle_increment;
		Eigen::Vector2d point_in_lidar;
		point_in_lidar.x() = range * std::cos(azimuth);
		point_in_lidar.y() = range * std::sin(azimuth);

		const double clustering_distance = range * angular_spacing_factor + 0.05;

		bool in_cluster = false;
		for ( auto & cluster : clusters )
		{
			if ( cluster.belongsBackPoint(point_in_lidar, clustering_distance) )
			{
				cluster.addPoint(point_in_lidar, 0.0);
				in_cluster = true;
				break;
			}
		}
		if ( !in_cluster )
			clusters.emplace_back(point_in_lidar, 0.0);
	}

	for ( auto & cluster : clusters )
	{
		if ( cluster.range() <= 0.0 )
			continue;

		const double column_aperture = 2.0 * std::atan2(column_size__ / 2.0, cluster.range());
		unsigned int min_support_points = static_cast<unsigned int>(std::floor(column_aperture / angle_increment_abs));
		if ( override_support_points__ > 0 )
			min_support_points = static_cast<unsigned int>(override_support_points__);
		else if ( min_support_points < 10 )
			min_support_points = 10;

		if ( cluster.supports() >= min_support_points && cluster.size() < column_size__ * 2.0 )
		{
			const Eigen::Vector2d corrected_centroid = cluster.centroid(column_size__ / 2.0);
			detections.emplace_back(
				cluster.supports(),
				corrected_centroid.x(),
				corrected_centroid.y(),
				column_size__ / 2.0);
		}
	}

	return detections;
}

}
