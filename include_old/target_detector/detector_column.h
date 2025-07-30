#ifndef TARGET_DETECTOR__DETECTOR_COLUMN_H
#define TARGET_DETECTOR__DETECTOR_COLUMN_H

// target_detector
#include "target_detector/cluster.h"
#include "target_detector/detector_lidar_2d.h"

namespace TargetDetector
{

class DetectorColumn : public DetectorLidar2d
{
	protected:
		double column_size__; //size for squared columns, diameter for circle columns, [m]
		double max_column_range__; // columns further away are ignored

	public:

		DetectorColumn();

		~DetectorColumn();

		// inits the detector with the parameters
		// - __params: set of tuning parameters: name and double value
		bool configure(const std::map<std::string, double> & __params);

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

};

}

#endif
