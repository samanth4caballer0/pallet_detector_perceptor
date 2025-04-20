# Overview

ROS "front-end" for object/marker detectors such as:

- Lidar reflector
- Column
- ALVAR single tag or bundle of tags (it is just a frame transform from ar_track_alvar output)
- Lidar geometric pattern, such as a v-marker (not implemented)
- Point cloud geometric pattern, such as a pallet (not implemented)

# Frames

The published detections are referenced to the platform frame. The X axis of the detected target object (marker, ...) is normal to the target object surface, pointing to the observer. The Z axis of the detected object is pointing up from the object, and the Y axis fulfills the right hand rule Z x X = Y.

# ROS API

### Parameters

~vizbose: if true, publishes visualization markers

~detector_type (uint8, no default):
  - 1 for DETECTOR_MARKER_REFLECTOR
  - 2 for DETECTOR_QR
  - 3 for DETECTOR_STD_PALLET (not implemented yet)

~robot_frame (string, no default): frame id for the platform. Output detections are referenced to this frame.

~lidar_frames (vector of strings, no default): names of frames of sensors

~min_reflector_intensity (double, [-], default 254): intensity threshold to consider a lidar hit to a reflector surface. May change a lot according the lidar device manufacturer.

~target_size (double, m, no default): One applies, depending on detector_type:
 - radius of a cylindric reflector
 - width of a planar reflector.
 - Side of a squared column

~max_target_range (double, [m], no default): range max to look for detections


### ROS interfaces

# RUN
