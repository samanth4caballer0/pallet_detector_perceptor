#include "target_detector/cluster.h"

int main(int argc, char **argv)
{
	std::cout << "Test Cluster" << std::endl;

	TargetDetector::Cluster c1(Eigen::Vector2d(1,2),100);
	c1.addPoint(Eigen::Vector2d(1.1,2.1),101);
	c1.addPoint(Eigen::Vector2d(0.95,1.95),104);
	c1.print(true);

	std::cout << "belongsCentroid(): " << c1.belongsCentroid(Eigen::Vector2d(1.01667+0.02, 2.01667+0.02), 0.05) << std::endl;
	std::cout << "belongsBackPoint(): " << c1.belongsBackPoint(Eigen::Vector2d(1.01667+0.02, 2.01667+0.02), 0.05) << std::endl;
	std::cout << "belongsAnyPoint(): " << c1.belongsAnyPoint(Eigen::Vector2d(1.+0.02, 2.+0.02), 0.05) << std::endl;
	std::cout << "belongsAnyPoint(): " << c1.belongsAnyPoint(Eigen::Vector2d(0.92,1.92), 0.05) << std::endl;
	std::cout << "belongsAnyPoint(): " << c1.belongsAnyPoint(Eigen::Vector2d(0.91,1.91), 0.01) << std::endl;

	return 0;
}
