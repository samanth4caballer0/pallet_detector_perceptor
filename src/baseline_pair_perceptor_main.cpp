#include "baseline_pair_perceptor.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "baseline_pair_perceptor");

	TargetDetector::BaselinePairPerceptor baseline_pair_perceptor;
	if ( !baseline_pair_perceptor.init() )
	{
		ROS_ERROR("Failed to init");
		return -1;
	}
	ROS_INFO("Inited");

	ros::spin();

	return 0;
}
