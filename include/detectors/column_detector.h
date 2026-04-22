#ifndef DETECTORS__COLUMN_DETECTOR_H
#define DETECTORS__COLUMN_DETECTOR_H

#include <cstdint>
#include <vector>

#include "detectors/cluster.h"

namespace Detectors
{

struct ColumnDetection
{
	uint32_t supports = 0;
	double centroid_x = 0.0;
	double centroid_y = 0.0;
	double covariance_xx = 0.25;
	double covariance_yy = 0.25;
	double nominal_radius = 0.0;

	ColumnDetection(
		const uint32_t & __supports,
		const double & __centroid_x,
		const double & __centroid_y,
		const double & __nominal_radius)
		: supports(__supports),
		  centroid_x(__centroid_x),
		  centroid_y(__centroid_y),
		  nominal_radius(__nominal_radius)
	{
	}
};

class ColumnDetector
{
	protected:

		double column_size__ = 0.5; // side length for square columns, diameter for cylindrical columns
		double max_detection_range__ = 25.0;
		double column_isolation_distance__ = 0.0;
		int override_support_points__ = 0;

	public:

		bool configure(
			const double & __column_size,
			const double & __max_detection_range,
			const double & __column_isolation_distance,
			const int & __override_support_points);

		std::vector<ColumnDetection> detect(
			const double & __angle_init,
			const double & __angle_end,
			const std::vector<float> & __ranges);
};

}

#endif
