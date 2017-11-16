# This is a message to hold data from an IMU (Inertial Measurement Unit) with a stamped message.
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#

std_msgs/Header header
localization_msgs/ImuLean imu 

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# angular rates x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
# Covariance is just the upper half of the covariance [x*x, x*y, x*z, y*y, y*z, z*z]
float64[6] angular_velocity_covariance

# linear accelerations x, y, z
# Covariance is just the upper half of the covariance [x*x, x*y, x*z, y*y, y*z, z*z]
float64[6] linear_acceleration_covariance
