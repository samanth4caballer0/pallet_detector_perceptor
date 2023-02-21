#include "marker_reflector/cluster.h"

namespace TargetDetector
{

Cluster::Cluster()
{
	//
}

Cluster::Cluster(const double & __px, const double & __py)
{
	addPoint(__px, __py);
}

Cluster::~Cluster()
{
	//
}

void Cluster::addPoint(const double & __point_x, const double & __point_y)
{
	points__.push_back(Eigen::Vector3d(__point_x, __point_y, 1.0)); //stored in homogeneous form
}

const unsigned int Cluster::size()
{
	return points__.size();
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
}

const Eigen::Vector3d & Cluster::centroid() const
{
	return centroid__;
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
}

} //namespace
