#include "detectors/column_detector.h"

#include <algorithm>
#include <cmath>

namespace Detectors
{

namespace
{
constexpr double kAutomaticSupportPointsRatio = 0.75;
constexpr unsigned int kMinimumAutomaticSupportPoints = 8;
}

bool ColumnDetector::configure(
	const double & __column_size,
	const double & __max_detection_range,
	const double & __column_isolation_distance,
	const int & __override_support_points)
{
	if ( !std::isfinite(__column_size) || __column_size <= 0.0 )
		return false;
	if ( !std::isfinite(__max_detection_range) || __max_detection_range <= 0.0 )
		return false;
	if ( !std::isfinite(__column_isolation_distance) || __column_isolation_distance < 0.0 )
		return false;
	if ( __override_support_points < 0 )
		return false;

	column_size__ = __column_size;
	max_detection_range__ = __max_detection_range;
	column_isolation_distance__ = __column_isolation_distance;
	override_support_points__ = __override_support_points;
	return true;
}

std::vector<ColumnDetection> ColumnDetector::detect(
	const double & __angle_init,
	const double & __angle_end,
	const std::vector<float> & __ranges)
{
	std::vector<ColumnDetection> detections;

	if ( __ranges.size() < 2 )
		return detections;

	const double angle_increment = (__angle_end - __angle_init) / static_cast<double>(__ranges.size() - 1);
	const double angle_increment_abs = std::abs(angle_increment);
	if ( !std::isfinite(angle_increment_abs) || angle_increment_abs <= 0.0 )
		return detections;

	const double angular_spacing_factor = 2.0 * std::tan(angle_increment_abs / 2.0);
	const double column_radius = column_size__ / 2.0;
	const double max_column_footprint_radius = column_size__ / std::sqrt(2.0);
	const double isolation_check_radius = max_column_footprint_radius + column_isolation_distance__;
	const double isolation_check_radius_sq = isolation_check_radius * isolation_check_radius;

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

	for ( size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index )
	{
		const Cluster & cluster = clusters[cluster_index];
		if ( cluster.range() <= 0.0 )
			continue;

		const Eigen::Vector2d corrected_centroid = cluster.centroid(column_radius);
		const double estimated_center_range = corrected_centroid.norm();
		if ( estimated_center_range <= column_radius )
			continue;

		const double column_aperture = 2.0 * std::asin(std::min(1.0, column_radius / estimated_center_range));
		const double theoretical_support_points = column_aperture / angle_increment_abs;
		unsigned int min_support_points = static_cast<unsigned int>(
			std::ceil(kAutomaticSupportPointsRatio * theoretical_support_points));
		if ( override_support_points__ > 0 )
			min_support_points = static_cast<unsigned int>(override_support_points__);
		else if ( min_support_points < kMinimumAutomaticSupportPoints )
			min_support_points = kMinimumAutomaticSupportPoints;

		bool isolated = true;
		if ( column_isolation_distance__ > 0.0 )
		{
			for ( size_t other_cluster_index = 0; other_cluster_index < clusters.size() && isolated; ++other_cluster_index )
			{
				if ( other_cluster_index == cluster_index )
					continue;

				for ( const auto & point : clusters[other_cluster_index].points() )
				{
					if ( (point - corrected_centroid).squaredNorm() <= isolation_check_radius_sq )
					{
						isolated = false;
						break;
					}
				}
			}
		}

		if ( cluster.supports() >= min_support_points && cluster.size() < column_size__ * 2.0 && isolated )
		{
			detections.emplace_back(
				cluster.supports(),
				corrected_centroid.x(),
				corrected_centroid.y(),
				column_radius);
		}
	}

	return detections;
}

}
