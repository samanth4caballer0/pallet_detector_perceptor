#ifndef TARGET_DETECTOR__CLUSTER_H
#define TARGET_DETECTOR__CLUSTER_H

#include <iostream>
#include <vector>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace TargetDetector
{

class Cluster
{
	protected:

		std::vector<Eigen::Vector2d> points__;
		std::vector<double> intensities__;
		Eigen::Vector2d centroid__;
		Eigen::Matrix2d covariance__;
		double intensity__;

	public:

		// constructor with a first point
		Cluster(const Eigen::Vector2d & __point, const double & __intensity);

		// returns a reference to the vector of points supporting this cluster
		const std::vector<Eigen::Vector2d> & points() const;

		// returns the number of points supporting this cluster
		unsigned int supports() const;

		// returns the number of points supporting this cluster
		double size() const;

		// returns a const reference to the centroid of this cluster
		const Eigen::Vector2d & centroid() const;

		// returns a copy of a corrected centroid in range
		Eigen::Vector2d centroid(const double & __delta_range) const;

		// returns mean intensity
		double intensity() const;

		// return the range of the centroid, [m]
		double range() const;

		// return the azimuth in [-pi, pi], [rad]
		double azimuth() const;

		// check if __point belongs to this cluster, according to the point to centroid distance
		bool belongsCentroid(const Eigen::Vector2d & __point, const double & __belonging_distance) const;

		// check if __point belongs to this cluster, according to the point to points__.back() distance
		bool belongsBackPoint(const Eigen::Vector2d & __point, const double & __belonging_distance) const;

		// check if __point belongs to this cluster, according to the point to any points__[ii] distance
		bool belongsAnyPoint(const Eigen::Vector2d & __point, const double & __belonging_distance) const;

		// adds a point to the cluster
 		// __point
		// __intensity
		void addPoint(const Eigen::Vector2d & __point, const double & __intensity);

		// transform centroid__ and covariance__ to another frame A
		// - __T: is the transform from frame A to current frame (i.e. lidar extrinsics: platform to lidar or lidar wrt platform)
		void transform(const Eigen::Isometry2d & __T);

		// prints cluster info
		// __verbose: if true, all points are printed
		void print(bool __verbose = false) const;

	protected:

		// recursively updates centroid and intensity
		void update();
};

}

#endif
