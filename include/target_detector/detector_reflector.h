#ifndef TARGET_DETECTOR__DETECTOR_REFLECTOR_H
#define TARGET_DETECTOR__DETECTOR_REFLECTOR_H

// target_detector
#include "target_detector/cluster.h"
#include "target_detector/detector_lidar_2d.h"

namespace TargetDetector
{

class DetectorReflector : public DetectorLidar2d
{
	protected:
		double min_reflector_intensity__; // intensity threshold
		double reflector_width__; // width of the reflector surface. If cylindrical, 40% of the diameter. [m]
		double max_reflector_range__; // reflectors further away are ignored

	public:

		DetectorReflector();

		~DetectorReflector();

		// inits the detector with the parameters
		// - __params: set of tuning parameters: name and double value
		bool configure(const std::map<std::string, double> & __params);

		// Returns detector type
		DetectorType type() const;

		// Detect reflectors.
		// Input argument details in DetectorLidar2d base class
		// Output argument __detections is a concatenation of 6 doubles: [size, intensity, x0,y0,cxx0,cyy0, ... ]
		bool detect(
			const double & __angle_init,
			const double & __angle_end,
			const std::vector<float> & __ranges,
			const std::vector<float> & __intensities,
			const Eigen::Isometry2d & __T_platform_sensor,
			std::vector<double> & __detections);

	protected:
		double correctDistortion(const double & __range);
};

}

#endif
