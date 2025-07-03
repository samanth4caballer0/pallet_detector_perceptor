# Overview

ROS "front-end" for object/marker detectors such as:

- Reflectors from lidar intensity
- Reflectors from bool
- Reflector pairs at baseline from single reflectors
- Columns from lidar scan
- ALVAR markers, single tag or bundle of tags (it is just a frame transform from ar_track_alvar output)
- Lidar geometric pattern, such as a v-marker (not implemented)
- Point cloud geometric pattern, such as a pallet (not implemented)

# Frames

The published detections are referenced to the platform frame. The X axis of the detected target object (marker, ...) is normal to the target object surface, pointing to the observer. The Z axis of the detected object is pointing up from the object, and the Y axis fulfills the right hand rule Z x X = Y.

# ROS API

### Parameters

~detector_type (uint8, no default):
  - TYPE_UNKNOWN = 0
  - TYPE_REFLECTOR_FROM_INTENSITY = 1
  - TYPE_REFLECTOR_FROM_BOOL = 2
  - TYPE_BASELINE_PAIR = 3
  - TYPE_COLUMN = 4
  - TYPE_STRAIGHT_SEGMENT = 5
  - TYPE_CORNER = 6
  - TYPE_PALLET = 7
  - TYPE_ALVAR = 8

~enable_at_init (bool, no default): whether the detector is enabled at init or not.

~robot_frame (string, no default): frame id for the platform. Output detections are referenced to this frame.

~vizbose (bool, no default): if true, publishes visualization markers

~lidar_frames (vector of strings, no default): names of frames of sensors

~min_reflector_intensity (double, [-], default 254): intensity threshold to consider a lidar hit to a reflector surface. May change a lot according the lidar device manufacturer.

~target_size (double, m, no default): One applies, depending on detector_type:
 - radius of a cylindric reflector
 - width of a planar reflector.
 - Side of a squared column

~max_target_range (double, [m], no default): range max to look for detections


### ROS interfaces

# RUN
