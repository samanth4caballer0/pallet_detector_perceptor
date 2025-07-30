#include "reflector_perceptor.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reflector_perceptor");

	TargetDetector::ReflectorPerceptor reflector_perceptor;
	if ( !reflector_perceptor.init() )
	{
		ROS_ERROR("Failed to init");
		return -1;
	}
	ROS_INFO("Inited");

	ros::spin();

	return 0;
}
