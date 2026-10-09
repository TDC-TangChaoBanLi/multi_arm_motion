# All timestamps use the shared ROS clock. trajectory.header.stamp is absolute start.
# Publish complete replacement predictions with a monotonically increasing version.
string source
uint64 version
builtin_interfaces/Time valid_until
trajectory_msgs/JointTrajectory trajectory
# Maximum expected absolute error for every predicted joint (radians or metres).
float64 position_uncertainty
