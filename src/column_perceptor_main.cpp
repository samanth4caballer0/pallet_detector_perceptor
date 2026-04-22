#include "column_perceptor.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "column_perceptor");

	TargetDetector::ColumnPerceptor column_perceptor;
	if ( !column_perceptor.init() )
	{
		ROS_ERROR("Failed to init");
		return -1;
	}
	ROS_INFO("Inited");

	ros::spin();

	return 0;
}
