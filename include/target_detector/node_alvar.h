#ifndef TARGET_DETECTOR_NODE_ALVAR_H
#define TARGET_DETECTOR_NODE_ALVAR_H

// std
#include <memory>

// ros
#include <visualization_msgs/Marker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

// this package
#include "target_detector/node_base.h"
#include <target_detector/target_detectorConfig.h>

namespace TargetDetector
{

class NodeAlvar : public NodeBase
{
	protected:

		// ROS API
		ros::Subscriber alvar_markers_subscriber__;
		ros::Publisher viz_marker_publisher__;

	public:

		NodeAlvar();
		~NodeAlvar();
		bool init();

	protected:

		// just transform to robot_frame and republish as alvars msg
		void alvarCallback(const ar_track_alvar_msgs::AlvarMarkers & __msg);
		void publishDetections(const std_msgs::Header & __header);
		void publishMarkers(const std_msgs::Header & __header);
};

}

#endif
