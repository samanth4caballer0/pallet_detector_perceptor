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

		// flag if the cluster parameters such as centroid have been updated
		bool updated__;

	public:
		Cluster();
		Cluster(const double & __px, const double & __py);
		~Cluster();

		// returns cluster size (number of points)
		const unsigned int size();

		// Return const ref to centroid__
		const Eigen::Vector3d & centroid();

		// Compute cluster parameters
		void compute();

		// Evaluate if a point may be member of this cluster
		bool evaluatePoint(const Eigen::Vector3d & __point, const double & __dist_threshold);

		//adds a new supporting laser point to the segment. Input arguments are the coordinates x,y of the point
		void addPoint(const double & __point_x, const double & __point_y);

		// Return euclidean distance from this centroid to argument's centroid
		double distance(Cluster & __cluster);

		// Check if distance from this to __cluster is equal to __dist argument with __epsilon tolerance
		bool checkDistance( Cluster & __cluster, double __dist,	double __epsilon);

		//print segment info
		void print() const;
};

} //namespace

#endif
