#include <detectors/cluster.h>

namespace Detectors
{

Cluster::Cluster(const Eigen::Vector2d & __point, const double & __intensity)
{
	addPoint(__point, __intensity);
}

const std::vector<Eigen::Vector2d> & Cluster::points() const
{
	return points__;
}

unsigned int Cluster::supports() const
{
	return points__.size();
}

double Cluster::size() const
{
	return (points__.back() - points__.front()).norm();
}

const Eigen::Vector2d & Cluster::centroid() const
{
	return centroid__;
}

Eigen::Vector2d Cluster::centroid(const double & __delta_range) const
{
	Eigen::Vector2d corrected_centroid;
	double range, azimuth;
	azimuth = this->azimuth();
	range = this->range();
	corrected_centroid.x() = (range + __delta_range)*std::cos(azimuth);
	corrected_centroid.y() = (range + __delta_range)*std::sin(azimuth);
	return corrected_centroid;
}

double Cluster::intensity() const
{
	return intensity__;
}

double Cluster::range() const
{
	return std::sqrt(centroid__.x()*centroid__.x() + centroid__.y()*centroid__.y());
}

double Cluster::azimuth() const
{
	return std::atan2(centroid__.y(), centroid__.x());
}

bool Cluster::belongsCentroid(const Eigen::Vector2d & __point, const double & __belonging_distance) const
{
	if ( points__.empty() )
		return false;

	double distance_squared = ((__point.x()-centroid__.x()) * (__point.x()-centroid__.x())) + ((__point.y()-centroid__.y()) * (__point.y()-centroid__.y()));
	return ( distance_squared < __belonging_distance*__belonging_distance );
}

bool Cluster::belongsBackPoint(const Eigen::Vector2d & __point, const double & __belonging_distance) const
{
	if ( points__.empty() )
		return false;

	double distance_squared = (__point - points__.back()).squaredNorm();
	return ( distance_squared < __belonging_distance*__belonging_distance );
}

bool Cluster::belongsAnyPoint(const Eigen::Vector2d & __point, const double & __belonging_distance) const
{
	if ( points__.empty() )
		return false;

	double distance_squared;
	bool belongs = false;
	for (unsigned int ii=0; ii<points__.size(); ii++)
	{
		distance_squared = ((__point.x()-points__[ii].x()) * (__point.x()-points__[ii].x())) + ((__point.y()-points__[ii].y()) * (__point.y()-points__[ii].y()));
		if ( distance_squared< __belonging_distance*__belonging_distance )
		{
			belongs = true;
			break;
		}
	}
	return belongs;
}

void Cluster::addPoint(const Eigen::Vector2d & __point, const double & __intensity)
{
	points__.push_back(__point);
	intensities__.push_back(__intensity);
	update();
}

void Cluster::transform(const Eigen::Isometry2d & __T)
{
	centroid__ = __T*centroid__; // transforms point to a new frame
	covariance__ = __T.linear()*covariance__;  // rotates covariance to a new frame
}

void Cluster::print(bool __verbose) const
{
	std::cout << "centroid: " << centroid__.transpose() << std::endl;
	std::cout << "size: " << this->size() << std::endl;
	std::cout << "intensity: " << intensity__ << std::endl;
	if (__verbose)
	{
		std::cout << "points (x y i): " << std::endl;
		for (unsigned int ii=0; ii<points__.size(); ii++)
		{
			std::cout << "\t" << points__[ii].transpose() << " " << intensities__[ii] << std::endl;
		}
	}
}

void Cluster::update()
{
	centroid__.x() = ( centroid__.x()*(points__.size()-1) + points__.back().x() ) / points__.size();
	centroid__.y() = ( centroid__.y()*(points__.size()-1) + points__.back().y() ) / points__.size();
	intensity__ = ( intensity__*(intensities__.size()-1) + intensities__.back() ) / intensities__.size();
}

}
