#include "target_detector/detector_column.h"

namespace TargetDetector
{

DetectorColumn::DetectorColumn()
{
	//
}

DetectorColumn::~DetectorColumn()
{
	//
}

bool DetectorColumn::configure(const std::map<std::string, double> & __params)
{
	// set config parameters
	if (__params.count("target_size") != 0 )
		column_size__ = __params.at("target_size");
	else
		column_size__ = 0.5;

	if (__params.count("max_target_range") != 0 )
		max_column_range__ = __params.at("max_target_range");
	else
		max_column_range__ = 25;

	return true;
}

DetectorType DetectorColumn::type() const
{
	return COLUMN;
}

bool DetectorColumn::detect(
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

	// precomputes fixed part involving tangent
	double angle_increment = ( __angle_end - __angle_init ) / __ranges.size();
	double tan_res_x2 = 2.*std::tan(std::fabs(angle_increment)/2.);
	//std::cout << "tan_res_x2: " << tan_res_x2 << std::endl;

	// do clustering
	std::vector<Cluster> clusters;
	bool in_cluster;
	double clustering_distance;
	double azimuth;
	Eigen::Vector2d point_in_lidar;
	for ( unsigned int ii = 0; ii < __ranges.size(); ii++ )
	{
		if ( __ranges[ii] < max_column_range__ )
		{
			// From polar to cartesian coordinates in sensor frame
			azimuth = __angle_init + ii*angle_increment;
			point_in_lidar.x() = __ranges[ii] * std::cos( azimuth );
			point_in_lidar.y() = __ranges[ii] * std::sin( azimuth );

			// computes clustering distance according scan data, +delta to take into account typical range noise
			clustering_distance = __ranges[ii]*tan_res_x2 + 0.05;

			// clustering
			in_cluster = false;
			for ( auto & cluster : clusters )
			{
				if ( cluster.belongsBackPoint(point_in_lidar, clustering_distance) )
				{
					cluster.addPoint(point_in_lidar, 0.0); // intensity 0, not taken into account here
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

	// Select clusters that can be columns
	double column_aperture;
	unsigned int min_support_points;
	Eigen::Vector2d corrected_centroid;
	for ( auto & cluster : clusters )
	{
		// compute min_support_points
		column_aperture = 2*std::atan2(column_size__/2.0 , cluster.range());
		min_support_points = std::floor(column_aperture / angle_increment );
		if (min_support_points < 10) min_support_points = 10;

		// Only select clusters with supports within [min_support_points, 2*min_support_points]
		if ( 	(cluster.supports() > min_support_points) &&
				(cluster.size() < column_size__*2 ) ) //  slightly greater than sqrt(2), to account for the diagonal
		{
			// transform to robot frame
			cluster.transform(__T_platform_sensor);
			__detections.push_back((double)cluster.supports());
			__detections.push_back(cluster.size());
			corrected_centroid = cluster.centroid(column_size__/2.); //center of the column further than cluster centroid by approx d/3
			__detections.push_back(corrected_centroid.x());
			__detections.push_back(corrected_centroid.y());
			__detections.push_back(0.25); //cxx not yet computed
			__detections.push_back(0.25); //cyy not yet computed

			// debug
			//cluster.print();
			//std::cout << "column_aperture: " << column_aperture*180./M_PI << std::endl;
			//std::cout << "min_support_points: " << min_support_points << std::endl;
		}


	}
	// debug
	//std::cout << "------------------------" << std::endl << std::endl;

	return true;
}


} // end of namespace
