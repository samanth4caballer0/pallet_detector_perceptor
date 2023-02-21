#ifndef TARGET_DETECTOR_CLUSTER_H
#define TARGET_DETECTOR_CLUSTER_H


// target_detector library
#include "marker_reflector/detector_base.h"
#include "marker_reflector/cluster.h"

namespace TargetDetector
{

class MarkerReflector : public DetectorBase
{
	protected:
		// data
		std::vector<Eigen::Vector3d> rpoints__; //points with positive reflectance, assumed ordered as given by the lidar
		std::vector<TargetDetector::Cluster> clusters__; // clusters of rpoints, according distance

		// parameters
		clustering_distance__;

	public:
		MarkerReflector();
		~MarkerReflector();
		bool init(const std::map<std::string, std::string> & __params);
		void resetData(); // clears both rpoints__ and clusters__
		bool detect(
			std::vector<Eigen::Vector3d> & __positions,
			std::vector<Eigen::Quaterniond> & __orientations,
		 	std::vector<double> & __confidences);
		void addReflectorPoint(const double & __x, const double & __y); //adds a point to rpoints__
};

} //namespace

#endif
