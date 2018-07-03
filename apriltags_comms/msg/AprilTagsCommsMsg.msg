# This message contains interrobot information from an observer to a receiver.
# There are a few frames of note:
#   map: the origin of the map. We wish to localise relative to this.
#   obs: the base_link of the observer.
#   tag: the identified tag of the receiver.
#   rec: the base_link of the receiver.
int32              obs_robot_id   # observer ID
string             tag_frame      # identified tag
nav_msgs/Odometry  map2obs        # observer EKF output
nav_msgs/Odometry  map2tag        # estimated location of the tag
nav_msgs/Odometry  obs2tag        # relative location of the tag

