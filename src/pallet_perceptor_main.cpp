#include "pallet_perceptor.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pallet_perceptor");

	TargetDetector::PalletPerceptor pallet_perceptor;
	if ( !pallet_perceptor.init() )
	{
		ROS_ERROR("Failed to init");
		return -1;
	}
	ROS_INFO("Inited");

	ros::spin();

	return 0;
}
