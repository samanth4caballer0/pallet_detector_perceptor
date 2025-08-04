#include "alvar_perceptor.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "alvar_perceptor");

	TargetDetector::AlvarPerceptor alvar_perceptor;
	if ( !alvar_perceptor.init() )
	{
		ROS_ERROR("Failed to init");
		return -1;
	}
	ROS_INFO("Inited");

	ros::spin();

	return 0;
}
