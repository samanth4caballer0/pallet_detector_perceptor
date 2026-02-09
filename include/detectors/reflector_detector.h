#ifndef DETECTORS__REFLECTOR_DETECTOR_H
#define DETECTORS__REFLECTOR_DETECTOR_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "detectors/cluster.h"

namespace Detectors
{

struct ReflectorDetection
{
	double supports = 0.0;
	double intensity = 0.0;
	double centroid_x = 0.0;
	double centroid_y = 0.0;
	double covariance_xx = 0.05;  // default value until computed
	double covariance_yy = 0.05;  // default value until computed

	ReflectorDetection(double __s, double __i, double __x, double __y)
		: supports(__s), intensity(__i), centroid_x(__x), centroid_y(__y) {}
};

class ReflectorDetector
{
	protected:

		double reflector_width__;
		double min_reflector_intensity__;
		double max_detection_range__;

	public:

		bool configure(const double & __reflector_size, const double & __min_reflector_intensity, const double & __max_detection_range);

		std::vector<ReflectorDetection> detect(const double & __angle_init, const double & __angle_end,
			const std::vector<float> & __ranges, const std::vector<float> & __intensities, const std::vector<uint8_t> & __reflector_hits, // empty if not used
			const Eigen::Isometry2d & __T_platform_sensor);

	protected:

		double correctDistortion(const double & __range);
};
	
}

#endif