#include "tmk_uwb_perceptor.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tmk_uwb_perceptor");

	TargetDetector::TmkUwbPerceptor tmk_uwb_perceptor;
	if ( !tmk_uwb_perceptor.init() )
	{
		ROS_ERROR("Failed to init");
		return -1;
	}
	ROS_INFO("Inited");

	ros::spin();

	return 0;
}
