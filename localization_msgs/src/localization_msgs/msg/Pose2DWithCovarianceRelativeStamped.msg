# This expresses an estimated position and orientation on a 2D manifold with a reference coordinate frame and timestamp

std_msgs/Header header
string child_frame_id

geometry_msgs/Pose2D pose2d

# Row-major representation of the 6x6 covariance matrix
# In order, the parameters are:
# position covariance x, y
# Covariance is just the upper half of the covariance [x*x, x*y, y*y]
float64[3] position_covariance

# Yaw covariance 
float64 yaw_covariance
