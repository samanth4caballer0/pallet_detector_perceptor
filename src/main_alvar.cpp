#include "target_detector/node_alvar.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "target_detector");

	TargetDetector::NodeAlvar target_detector;
	if ( !target_detector.init() )
	{
		ROS_ERROR("Failed to init");
		return -1;
	}
	ROS_INFO("Inited");

	ros::spin();

	return 0;
}
