#include "barrel_perceptor.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "barrel_perceptor");

	TargetDetector::BarrelPerceptor barrel_perceptor;
	if ( !barrel_perceptor.init() )
	{
		ROS_ERROR("Failed to init");
		return -1;
	}
	ROS_INFO("Inited");

	ros::spin();

	return 0;
}
