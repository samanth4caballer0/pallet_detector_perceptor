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
	reflector_distance__ = __params.at("reflector_distance");
	distance_tolerance__ = __params.at("distance_tolerance");
	return true;
}

void MarkerReflector::resetData()
{
	rpoints__.clear();
	clusters__.clear();
}

bool MarkerReflector::detect(
	std::vector<Eigen::Vector3d> & __key_points,
	std::vector<Eigen::Vector3d> & __positions,
	std::vector<Eigen::Quaterniond> & __orientations,
	std::vector<double> & __confidences)
{
	// Initial check
	if (rpoints__.size() < 2)
		return false;

	// clustering
	clusters__.push_back( TargetDetector::Cluster(rpoints__[0].x(), rpoints__[0].y()) ); //at least one cluster with the firts point
	for (unsigned int ii=1; ii<rpoints__.size(); ii++)
	{
		for (unsigned int jj=0; jj<clusters__.size(); jj++ )
		{
			if ( clusters__[jj].evaluatePoint(rpoints__[ii], clustering_distance__) )
			{
				clusters__.back().addPoint(rpoints__[ii].x(), rpoints__[ii].y());
				break;
			}
		}
		clusters__.push_back(TargetDetector::Cluster());
		clusters__.back().addPoint(rpoints__[ii].x(), rpoints__[ii].y());
	}

	// Prune spurious clusters (if only 1 point support)
	for(std::std::vector<TargetDetector::Cluster>::iterator it = clusters__.begin(); it != clusters__.end()) // it increment inside the loop
	{
		if ( it->size() < 2 )
			it = clusters__.erase(cluster_set.begin()+ii);
		else
			it ++;
	}

	// Set __key_points with cluster centroids
	for(unsigned int ii=0; ii<clusters__.size(); ii++)
	{
		__key_points.push_back( Eigen::Vector3d(clusters__[ii].centroid()) );
	}

	// check on number of clusters
	if (clusters__.size() < 2)
		return false;

	// compute distances between all cluster pairs (only upper triangle)
	Eigen::MatrixXd d_matrix(clusters__.size(), clusters__.size());
	for (unsigned int ii = 0; ii<clusters__.size(); ii++)
	{
		for (unsigned int jj = ii+1; jj<clusters__.size(); jj++)
		{
			d_matrix(ii,jj) = (clusters__[ii].centroid() - clusters__[jj].centroid()).norm();
		}
	}

	// Keep only the pairs matching the correct distance
	std::vector<std::pair<unsigned int, unsigned int> > candidates_ij;
	for (unsigned int ii = 0; ii<clusters__.size(); ii++)
	{
		for (unsigned int jj = ii+1; jj<clusters__.size(); jj++)
		{
			if ( std::abs(d_matrix(ii,jj) - reflector_distance__) < distance_tolerance__ )
			{
				candidates_ij.push_back(std::pair<unsigned int, unsigned int>());
				candidates_ij.back().first = ii;
				candidates_ij.back().second = jj;
			}
		}
	}

	// Compute marker frames for each candidate pair
	Eigen::Vector3d vx,vy;
	Eigen::Vector3d vz(0,0,1);
	for( const auto & it : candidates_ij )
	{
		vy = clusters__[candidates_ij.first].centroid() - clusters__[candidates_ij.second].centroid();
		vx = vy.cross(vz);
		
	}

	return true;


}

void MarkerReflector::addReflectorPoint(const double & __x, const double & __y)
{
	rpoints__.push_back(Eigen::Vector3d(__x, __y, 1.0));
}
