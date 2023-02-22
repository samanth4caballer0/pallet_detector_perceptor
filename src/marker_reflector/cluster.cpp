#include "marker_reflector/cluster.h"

namespace TargetDetector
{

Cluster::Cluster()
{
	updated__ = false;
}

Cluster::Cluster(const double & __px, const double & __py)
{
	addPoint(__px, __py);
}

Cluster::~Cluster()
{
	//
}

const unsigned int Cluster::size()
{
	return points__.size();
}

const Eigen::Vector3d & Cluster::centroid() const
{
	if ( !updated__ ) compute();
	return centroid__;
}

void Cluster::compute()
{
	// check someone in the cluster
	if ( points__.empty() ) return;

	// centroid
	double xx(0), yy(0);
	for (unsigned int ii=0; ii<points__.size(); ii++)
	{
		xx += points__[ii].x();
		yy += points__[ii].y();
	}
	centroid__ << xx/points__.size(), yy/points__.size(), 1.0;

	updated__ = true;
}

bool evaluatePoint(const Eigen::Vector3d & __point, const double & __dist_threshold)
{
	for (unsigned int ii=0; ii<points__.size(); ii++)
	{
		if ( (points__[ii] - __point).squaredNorm() < __dist_threshold*__dist_threshold )
		{
			return true;
		}
	}
	return false; 
}

void Cluster::addPoint(const double & __point_x, const double & __point_y)
{
	points__.push_back(Eigen::Vector3d(__point_x, __point_y, 1.0)); //stored in homogeneous form
	updated__ = false;
}

double Cluster::distance(const Cluster & __cluster) const
{
	return sqrt(
		( centroid__.x() - __cluster.centroid().x() ) * ( centroid__.x() - __cluster.centroid().x() )
		+
		( centroid__.y() - __cluster.centroid().y() ) * ( centroid__.y() - __cluster.centroid().y() ) );
}

bool Cluster::checkDistance( const Cluster & __cluster, double __dist, double __epsilon) const
{
	double dd = std::fabs( distance(__cluster) - __dist );
	if ( dd < __epsilon ) return true;
	else return false;
}


void Cluster::print() const
{
	std::cout << "Cluster" << std::endl;
	std::cout << "\t Size: " << points__.size() << std::endl;
	std::cout << "\t Centroid: " << centroid__.transpose() << std::endl;
	std::cout << "\t Updated: " << (unsigned int)updated__ << std::endl;
}

} //namespace
