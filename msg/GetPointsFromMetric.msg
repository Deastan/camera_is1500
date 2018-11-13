# GetPointsFromMetric.msg
# Contain the position and yaw which are detected from the metric
# Contain the cost of the detection

# Position
float64 x
float64 y
float64 z

# Orientation
float64 yaw

# Cost from the metric
float64 cost
int32 mapNumber
