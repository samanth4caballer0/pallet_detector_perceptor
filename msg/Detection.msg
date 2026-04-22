uint8 type
uint8 UNKNOWN = 0
uint8 REFLECTOR_FROM_INTENSITY = 1
uint8 REFLECTOR_FROM_BOOL = 2
uint8 BASELINE_PAIR = 3
uint8 ALVAR = 4
uint8 TMK_UWB = 5
uint8 VERTICAL_CYLINDER = 6
uint8 COLOR_IN_ROI = 7
uint8 COLUMN = 8

# some detections have an associated id (AR markers, object class ,... )
# If no id, set to -1
int32 id

# pose and orientation of the detection
geometry_msgs/PoseWithCovariance pose

# mean intensity for reflector detection
float64 intensity

# number of support points for cluster based detectors
uint32 supports

# support points for cluster based detectors
geometry_msgs/Point[] points

# baseline for reflector pair detector
float64 baseline

# radius for cylinder-like or column detections
float64 radius

# color used by color detectors
target_detector/Color color
