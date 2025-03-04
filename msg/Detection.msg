# detection type
uint32 type

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
