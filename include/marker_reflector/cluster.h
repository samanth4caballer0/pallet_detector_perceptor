#ifndef TARGET_DETECTOR_CLUSTER_H
#define TARGET_DETECTOR_CLUSTER_H

//std
#include <iostream>
#include <vector>
#include <cmath>

//Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace TargetDetector
{

class Cluster
{
	protected:
		//Each column is the homogeneous coordinates of a 2D point supporting the cluster. Ordered as the scan.
		std::vector<Eigen::Vector3d> points__;

		// centroid of the cluster in homogeneous coordinates
		Eigen::Vector3d centroid__;

	public:
		//constructors
		Cluster();
		Cluster(const double & __px, const double & __py);

		//destructor
		~Cluster();

		//adds a new supporting laser point to the segment. Input arguments are the coordinates x,y of the point
		void addPoint(const double & __point_x, const double & __point_y);

		// returns cluster size (number of points)
		const unsigned int size();

		// Compute cluster parameters
		void compute();

		// Return const ref to centroid__
		const Eigen::Vector3d & centroid() const;

		// Return euclidean distance from this centroid to argument's centroid
		double distance(const Cluster & __cluster) const;

		// Check if distance from this to __cluster is equal to __dist argument with __epsilon tolerance
		bool checkDistance( const Cluster & __cluster, double __dist,	double __epsilon) const;

		//print segment info
		void print() const;
};

} //namespace

#endif
