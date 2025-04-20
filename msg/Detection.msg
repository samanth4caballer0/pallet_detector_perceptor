uint8 TYPE_UNKNOWN = 0
uint8 TYPE_REFLECTOR_FROM_INTENSITY = 1
uint8 TYPE_REFLECTOR_FROM_BOOL = 2
uint8 TYPE_REFLECTOR_PAIR = 3
uint8 TYPE_COLUMN = 4
uint8 TYPE_STRAIGHT_SEGMENT = 5
uint8 TYPE_CORNER = 6
uint8 TYPE_PALLET = 7
uint8 TYPE_ALVAR = 8

# detection type
uint8 type

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
