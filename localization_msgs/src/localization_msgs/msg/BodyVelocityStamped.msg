# This message is meant to hold information relating motion in the sensor frame
# on the robot body. The direction of the motion reported is relative to where 
# the sensor was before.
# Note that this message copies the contents of geometry_msgs/TwistStamped

std_msgs/Header header
geometry_msgs/Twist twist