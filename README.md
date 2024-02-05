# Overview

ROS "front-end" for object/marker detectors such as:

- Lidar reflector marker
- AR tag
- Lidar geometric pattern, such as a v-marker (not implemented)
- Point cloud geometric pattern, such as a pallet (not implemented)

# ROS API

### Parameters

~type (int, no default):
  - 1 for DETECTOR_MARKER_REFLECTOR
  - 2 for DETECTOR_QR
  - 3 for DETECTOR_STD_PALLET (not implemented yet)

~lidar_frames (vector of strings, no default): names of frames of sensors

~topic_type (int, default 1): for laser based detectors, either
   - 1 for sensor_msgs/LaserScan
   - 2 for sick_safetyscanners/ExtendedLaserScanMsg

~reflector_intensity_threshold (double, [-], default 254): intensity threshold to consider a lidar hit to a reflector surface. May change a lot according the lidar device manufacturer.

~clustering_distance (double, [m], default 0.1): max distance between points (scan or cloud) to be considered in the same cluster.

~reflector_distance_tolerance (double, [m], default 0.03): max difference between expected and measured pattern distance between reflector elements.


### ROS interfaces

# RUN
