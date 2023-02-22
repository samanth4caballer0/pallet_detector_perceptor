#ifndef TARGET_DETECTOR_DETECTOR_BASE_H
#define TARGET_DETECTOR_DETECTOR_BASE_H

//std
#include <iostream>
#include <map>

//Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace TargetDetector
{

class DetectorBase
{
	protected:

	public:
		DetectorBase();
		~DetectorBase();

		// initialization with custom parameters, depending on each class specialization
		virtual bool init(const std::map<std::string, std::string> & __params) = 0;

		// clears input sensory data stored in class members, to start a new detection
		virtual void resetData();

		// detect method only defines output arguments. Input data depending on the class specialization
		virtual bool detect(
			std::vector<Eigen::Vector3d> & __key_points, // key points for the detector, useful for visualization
			std::vector<Eigen::Vector3d> & __positions, // origin of marker frames
			std::vector<Eigen::Quaterniond> & __orientations, // orientation of marker frames
		 	std::vector<double> & __confidences) = 0;
};

} //namespace

#endif
