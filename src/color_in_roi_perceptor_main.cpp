#include "color_in_roi_perceptor.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "color_in_roi_perceptor");

	TargetDetector::ColorInRoiPerceptor color_in_roi_perceptor;
	if ( !color_in_roi_perceptor.init() )
	{
		ROS_ERROR("Failed to init");
		return -1;
	}
	ROS_INFO("Inited");

	ros::spin();

	return 0;
}
