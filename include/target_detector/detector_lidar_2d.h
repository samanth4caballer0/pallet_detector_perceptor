#ifndef TARGET_DETECTOR__DETECTOR_LIDAR_2D_H
#define TARGET_DETECTOR__DETECTOR_LIDAR_2D_H

// eigen
#include "target_detector/detector_base.h"

namespace TargetDetector
{

class DetectorLidar2d : public DetectorBase
{
	protected:

	public:

		DetectorLidar2d(){};

		~DetectorLidar2d(){};

		// Configure the detector with the parameters
		// - __params: set of tuning parameters: name and value
		virtual bool configure(const std::map<std::string, double> & __params) = 0;

		// Returns detector type
		virtual DetectorType type() const = 0;

		// detect targets in range scan data
		// - __angle_init: angle of the first scan point [rad]
		// - __angle_end: angle of the last scan point [rad]
		// - __ranges: scan range data [m]
		// - __intensities: scan intensity data [*]
		// - __T_platform_sensor: Homogeneous tranform from platform to sensor (sensor rwt platform)
		// - __detection result. Defined at each derived class
		virtual bool detect(
			const double & __angle_init,
			const double & __angle_end,
			const std::vector<float> & __ranges,
			const std::vector<float> & __intensities,
			const Eigen::Isometry2d & __T_platform_sensor,
			std::vector<double> & __detections) = 0;
};

}

#endif
