#ifndef TARGET_DETECTOR__DETECTOR_BASE_H
#define TARGET_DETECTOR__DETECTOR_BASE_H

// eigen
#include <iostream>
#include <map>

namespace TargetDetector
{

enum DetectorType
{
	UNKNOWN = 0,
	REFLECTOR,
	REFLECTOR_PAIR,
	COLUMN,
	STRAIGHT_SEGMENT,
	CORNER,
	PALLET,
	ALVAR
};


class DetectorBase
{
	protected:

	public:

		DetectorBase(){};
		~DetectorBase(){};

		// Configures the detector with the parameters
		// - __params: set of tuning parameters: name and value
		virtual bool configure(const std::map<std::string, double> & __params) = 0;

		// Returns detector type
		virtual DetectorType type() const = 0;
};

}

#endif
