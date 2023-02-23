#ifndef TARGET_DETECTOR_MARKER_REFLECTOR_H
#define TARGET_DETECTOR_MARKER_REFLECTOR_H


// target_detector library
#include "target_detector/detector_base.h"
#include "marker_reflector/cluster.h"

namespace TargetDetector
{

class MarkerReflector : public DetectorBase
{
	protected:
		// data
		std::vector<Eigen::Vector3d> rpoints__; //points with positive reflectance, wrt the platform frame. Ordering not assumed.
		std::vector<Cluster> clusters__; // clusters of rpoints

		// parameters
		double clustering_distance__;
		double reflector_distance__;
		double reflector_distance_tolerance__;

	public:
		MarkerReflector();
		~MarkerReflector();
		bool init(const std::map<std::string, std::string> & __params);
		void resetData(); // clears both rpoints__ and clusters__
		bool detect(
			std::vector<Eigen::Vector3d> & __key_points,
			std::vector<Eigen::Vector3d> & __positions,
			std::vector<Eigen::Quaterniond> & __orientations,
		 	std::vector<double> & __confidences);
		void addPointData(
			const double & __x,
			const double & __y,
			const double & __z);
};

} //namespace

#endif
