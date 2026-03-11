#include "vertical_cylinder_perceptor.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vertical_cylinder_perceptor");

	TargetDetector::VerticalCylinderPerceptor vertical_cylinder_perceptor;
	if ( !vertical_cylinder_perceptor.init() )
	{
		ROS_ERROR("Failed to init");
		return -1;
	}
	ROS_INFO("Inited");

	ros::spin();

	return 0;
}
