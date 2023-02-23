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
	clustering_distance__ = std::stod(__params.at("clustering_distance"));
	reflector_distance__ = std::stod(__params.at("reflector_distance"));
	reflector_distance_tolerance__ = std::stod(__params.at("reflector_distance_tolerance"));
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
	for( std::vector<TargetDetector::Cluster>::iterator it = clusters__.begin(); it != clusters__.end(); ) // it increment inside the loop
	{
		if ( it->size() < 2 )
			it = clusters__.erase(it);
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
			if ( std::abs(d_matrix(ii,jj) - reflector_distance__) < reflector_distance_tolerance__ )
			{
				candidates_ij.push_back(std::pair<unsigned int, unsigned int>());
				candidates_ij.back().first = ii;
				candidates_ij.back().second = jj;
			}
		}
	}

	// Compute marker frames for each candidate pair
	Eigen::Vector3d vx,vy; // frame vectors x and y
	Eigen::Vector3d vz(0,0,1); // frame vector z
	Eigen::Vector3d mp; //vector from marker origin to platform origin
	double dot_p;
	Eigen::Quaterniond qt;
	double angle_z;
	qt.x() = 0.0;
	qt.y() = 0.0;
	for( const auto & it : candidates_ij )
	{
		vy = clusters__[it.first].centroid() - clusters__[it.second].centroid();
		vx = vy.cross(vz);
		mp = clusters__[it.first].centroid();
		mp.z() = 0;
		dot_p = vx.dot(mp);
		if ( dot_p < 0.0 ) // check frame alignement
		{
			vy = clusters__[it.second].centroid() - clusters__[it.first].centroid();
			vx = vy.cross(vz);
			mp = clusters__[it.second].centroid();
		}
		angle_z = atan2(vx.y(),vx.x());
		qt.z() = sin(angle_z/2.0);
		qt.w() = cos(angle_z/2.0);
		__positions.push_back(mp);
		__orientations.push_back(qt);
		__confidences.push_back(1.0);
	}

	return true;
}

void MarkerReflector::addPointData(
	const double & __x,
	const double & __y,
	const double & __z)
{
	rpoints__.push_back(Eigen::Vector3d(__x, __y, 1.0));
}

} // end of namespace
