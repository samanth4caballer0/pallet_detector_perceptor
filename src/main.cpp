#include "target_detector/node.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "target_detector");

	TargetDetector::Node target_detector;
	if ( !target_detector.init() )
	{
		ROS_ERROR("Failed to init");
		return -1;
	}
	ROS_INFO("Inited");

	ros::spin();

	return 0;
}
