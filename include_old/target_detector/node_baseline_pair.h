#ifndef TARGET_DETECTOR_NODE_BASELINE_PAIR_H
#define TARGET_DETECTOR_NODE_BASELINE_PAIR_H

// std
#include <memory>

// ros
#include <ar_track_alvar_msgs/AlvarMarkers.h>

// this package
#include "target_detector/node_base.h"
#include <target_detector/target_detectorConfig.h>

namespace TargetDetector
{

class NodeBaselinePair : public NodeBase
{
	protected:

		// ROS API
		ros::Subscriber detections_subscriber__;

	public:

		NodeBaselinePair();
		~NodeBaselinePair();
		bool init();

	protected:

		void detectionsCallback(const target_detector::Detections & __msg);
		void publishMarkers(const target_detector::Detections & __msg);
};

}

#endif
