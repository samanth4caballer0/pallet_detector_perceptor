#include "target_detector/target_detector_node.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "target_detector");

	TargetDetector::TargetDetectorNode node;
	if ( !node.init() )
	{
		ROS_ERROR("Failed to init");
		return -1;
	}
	ROS_INFO("Inited");

	ros::spin();

	return 0;
}
