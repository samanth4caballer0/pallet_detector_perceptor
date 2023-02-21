#include "marker_reflector/marker_reflector.h"

namespace TargetDetector
{

MarkerReflector::MarkerReflector()
{

}

MarkerReflector::~MarkerReflector()
{

}

bool MarkerReflector::init(const std::map<std::string, std::string> & __params)
{
	clustering_distance__ = __params.at("clustering_distance");
	return true;
}

void MarkerReflector::resetData()
{
	rpoints__.clear();
	clusters__.clear();
}

bool MarkerReflector::detect(
	std::vector<Eigen::Vector3d> & __positions,
	std::vector<Eigen::Quaterniond> & __orientations,
	std::vector<double> & __confidences)
{
	// Initial check
	if (rpoints__.size() < 2)
		return false;

	// clustering
	double d2;
	clusters__.push_back( TargetDetector::Cluster(rpoints__[0].x(), rpoints__[0].y()) ); //at least one cluster with the firts point
	for (unsigned int ii=1; ii<rpoints__.size(); ii++)
	{
		d2 = (rpoints__[ii-1]-rpoints__[ii]).squaredNorm();
		if ( d2 > clustering_distance__*clustering_distance__ ) //condition for a new cluster
		{
			clusters__.push_back(TargetDetector::Cluster());
		}
		clusters__.back().addPoint(rpoints__[ii].x(), rpoints__[ii].y());
	}

	// check on number of clusters
	if (clusters__.size() < 2)
		return false;

	// TODO ...

	return true;


}

void MarkerReflector::addReflectorPoint(const double & __x, const double & __y)
{
	rpoints__.push_back(Eigen::Vector3d(__x, __y, 1.0));
}
