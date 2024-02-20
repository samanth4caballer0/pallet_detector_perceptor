#include "target_detector/target_detector.h"

namespace TargetDetector
{

bool TargetDetector::init()
{
	// get config params
	if ( !nh__.getParam("reflector_distance_tolerance", reflector_distance_tolerance__) )
	{
		ROS_WARN("Failed to get reflector_distance_tolerance parameter. Setting to default value 0.03.");
		reflector_distance_tolerance__ = 0.03;
	}

	// init ros api
	mode_server__ = nh__.advertiseService("detector_mode", &TargetDetector::modeCallback, this);
	reflector_subscriber__ = nh__.subscribe("reflectors", 1, &TargetDetector::reflectorCallback, this);
	alvar_subscriber__ = nh__.subscribe("alvar", 1, &TargetDetector::alvarCallback, this);
	detector_publisher__ = nh__.advertise<target_detector::Detections>( "target_detections", 1, false );

	// init mode
	mode__ = target_detector::Detector::Request::IDLE;

	// init tf
	tf_listener__.reset(new tf2_ros::TransformListener(tf_buffer__));

	return true;
}

bool TargetDetector::modeCallback(target_detector::Detector::Request & __request, target_detector::Detector::Response & __response)
{
	switch (__request.mode)
	{
		case target_detector::Detector::Request::IDLE:
			mode__ = target_detector::Detector::Request::IDLE;
			__response.success = true;
			break;

		case target_detector::Detector::Request::REFLECTOR_MARKERS:
			if ( __request.reflector_baseline < 0.0)
			{
				ROS_WARN("TargetDetector::modeCallback: reflector baseline must be positive. Detector mode set to idle");
				mode__ = target_detector::Detector::Request::IDLE;
				__response.success = false;
			}
			else
			{
				mode__ = target_detector::Detector::Request::REFLECTOR_MARKERS;
				reflector_baseline__ = __request.reflector_baseline;
				__response.success = true;
			}
			break;

		case target_detector::Detector::Request::ALVAR_MARKERS:
			if ( (__request.alvar_marker_id < 0) || (__request.alvar_marker_id > 18 ) )
			{
				ROS_WARN("TargetDetector::modeCallback: alvar marker id must be in [0,18]. Detector mode set to idel");
				mode__ = target_detector::Detector::Request::IDLE;
				__response.success = false;
			}
			else
			{
				mode__ = target_detector::Detector::Request::ALVAR_MARKERS;
				alvar_marker_id__ = __request.alvar_marker_id;
				__response.success = true;
			}
			break;

		default:
			ROS_WARN("TargetDetector::modeCallback: Unknown detector mode. Detector mode set to idle.");
			mode__ = target_detector::Detector::Request::IDLE;
			__response.success = false;
			break;
	}

	return true;
}

void TargetDetector::reflectorCallback(const reflector_finder::Reflectors & __reflectors)
{
	target_detector::Detections detections;
	target_detector::Detection detection;
	double angle;

	detections.header = __reflectors.header;
	if ( 	( __reflectors.reflectors.empty() ) ||
			( mode__ != target_detector::Detector::Request::REFLECTOR_MARKERS ) )
	{
		detector_publisher__.publish(detections);
		return;
	}

	// check all reflector pairs for correct baseline
	for ( int i = 0; i < __reflectors.reflectors.size(); i++)
	{
		for ( int j = i+1; j < __reflectors.reflectors.size(); j++)
		{
			Eigen::Vector3d reflector_one(__reflectors.reflectors.at(i).centroid.x, __reflectors.reflectors.at(i).centroid.y, 1.0);
			Eigen::Vector3d reflector_two(__reflectors.reflectors.at(j).centroid.x, __reflectors.reflectors.at(j).centroid.y, 1.0);

			double actual_baseline = computeBaseline(reflector_one, reflector_two);
			if ( std::abs( actual_baseline - reflector_baseline__ ) < reflector_distance_tolerance__ )
			{
				// we have a detection, so compute marker frame
				Eigen::Vector3d x_axis;
				Eigen::Vector3d y_axis;
				Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
				Eigen::Vector3d platform_to_marker;

				y_axis = reflector_two - reflector_one;
				x_axis = y_axis.cross(z_axis);
				platform_to_marker = reflector_one;
				if ( x_axis.dot(platform_to_marker) > 0.0 ) // we need to switch axes
				{
					y_axis = -y_axis;
					x_axis = y_axis.cross(z_axis);
					platform_to_marker = reflector_two;
				}

				detection.pose.position = tf2::toMsg(platform_to_marker);
				angle = std::atan2(x_axis.y(), x_axis.x());
				detection.pose.orientation.x = 0.;
				detection.pose.orientation.y = 0.;
				detection.pose.orientation.z = std::sin(angle/2.0);
				detection.pose.orientation.w = std::cos(angle/2.0);
				detection.supports.push_back(__reflectors.reflectors.at(i).centroid);
				detection.supports.push_back(__reflectors.reflectors.at(j).centroid);
				detection.baseline = actual_baseline;

				detections.detections.push_back(detection);
			}
		}
	}
	detector_publisher__.publish(detections);
}

void TargetDetector::alvarCallback(const ar_track_alvar_msgs::AlvarMarkers & __alvar_markers)
{
	target_detector::Detections detections;
	target_detector::Detection detection;

	// idle or empty case ...
	detections.header = __alvar_markers.header; //this top message header is always empty
	if ( 	( __alvar_markers.markers.empty() ) ||
			( mode__ != target_detector::Detector::Request::ALVAR_MARKERS ) )
	{
		detector_publisher__.publish(detections);
		return;
	}

	// if at least on marker, get header from first marker detected, since top message header is always empty
	detections.header = __alvar_markers.markers[0].header;

	// working case ...
	geometry_msgs::TransformStamped tr_st;
	Eigen::Quaterniond qt; //auxiliar quaternion
	Eigen::Isometry3d Tp_c; // camera wrt platform (platform to camera). Static, from TF
	Eigen::Isometry3d Tc_m; // marker wrt camera (camera to marker). From alvar detector
	Eigen::Isometry3d Tp_m; // marker wrt platform (platform to marker). To be computed and published
	double angle;
	for ( int ii = 0; ii < __alvar_markers.markers.size(); ii++)
	{
		if ( __alvar_markers.markers[ii].id == alvar_marker_id__ )
		{
			// transform detection from camera to platform frame

			// build platform to camera
			try
			{
				tr_st = tf_buffer__.lookupTransform("platform", __alvar_markers.markers[0].header.frame_id,ros::Time(0));
			}
			catch (tf2::TransformException &ex)
			{
				ROS_WARN("%s",ex.what());
				return;
			}
			qt.coeffs() << 	tr_st.transform.rotation.x,
							tr_st.transform.rotation.y,
							tr_st.transform.rotation.z,
							tr_st.transform.rotation.w;
			Tp_c.linear() = qt.matrix();
			Tp_c.translation() << 	tr_st.transform.translation.x,
									tr_st.transform.translation.y,
									tr_st.transform.translation.z;

			// build camera to marker
			qt.coeffs() << 	__alvar_markers.markers[ii].pose.pose.orientation.x,
							__alvar_markers.markers[ii].pose.pose.orientation.y,
							__alvar_markers.markers[ii].pose.pose.orientation.z,
							__alvar_markers.markers[ii].pose.pose.orientation.w;
			Tc_m.linear() = qt.matrix();
			Tc_m.translation() <<	__alvar_markers.markers[ii].pose.pose.position.x,
									__alvar_markers.markers[ii].pose.pose.position.y,
									__alvar_markers.markers[ii].pose.pose.position.z;

			// compute platform to marker and fill the output message
			Tp_m = Tp_c*Tc_m;
			detection.pose.position.x = Tp_m.translation().x();
			detection.pose.position.y = Tp_m.translation().y();
			detection.pose.position.z = Tp_m.translation().z();
			angle = atan2(Tp_m.linear()(1,2),Tp_m.linear()(0,2)); // angle of marker Z axis wrt platform, in the XY platform plane
			detection.pose.orientation.x = 0.;
			detection.pose.orientation.y = 0.;
			detection.pose.orientation.z = std::sin(angle/2.0);
			detection.pose.orientation.w = std::cos(angle/2.0);
			detections.detections.push_back(detection);
		}
	}

	detector_publisher__.publish(detections);
}

double TargetDetector::computeBaseline(const Eigen::Vector3d & __reflector_one, const Eigen::Vector3d & __reflector_two)
{
	return	(__reflector_one-__reflector_two).norm();
}


}
