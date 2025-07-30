#ifndef TARGET_DETECTOR__DETECTOR_BASE_H
#define TARGET_DETECTOR__DETECTOR_BASE_H

// std
#include <iostream>
#include <string>
#include <map>

// eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace TargetDetector
{

class DetectorBase
{
	protected:

	public:

		DetectorBase(){};
		~DetectorBase(){};

		// Configures the detector with the parameters
		// - __params: set of tuning parameters: name and value
		virtual bool configure(const std::map<std::string, double> & __params) = 0;
};

}

#endif
