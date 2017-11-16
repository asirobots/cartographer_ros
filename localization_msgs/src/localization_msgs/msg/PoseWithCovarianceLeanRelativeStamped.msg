# This expresses an estimated pose with a reference coordinate frame and timestamp

std_msgs/Header header
string child_frame_id

geometry_msgs/Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# position covariance x, y, z
# Covariance is just the upper half of the covariance [x*x, x*y, x*z, y*y, y*z, z*z]
float64[6] position_covariance

# Yaw/Pitch/Roll Convention orientation covariance 
# Covariance is just the upper half of the covariance [roll*roll, roll*pitch, roll*yaw, pitch*pitch, pitch*yaw, yaw*yaw]
float64[6] orientation_covariance
