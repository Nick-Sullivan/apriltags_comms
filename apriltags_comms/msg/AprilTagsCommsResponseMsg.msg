# This message contains interrobot information from the receiver to the observer.
# There are a few frames of note:
#   map: the origin of the map. We wish to localise relative to this.
#   obs: the base_link of the observer.
#   rec: the base_link of the receiver.
int32              obs_robot_id    # observer_id
nav_msgs/Odometry  map2obs         # estimated location of the observer
nav_msgs/Odometry  obs2rec         # relative location of the receiver
nav_msgs/Odometry  rec2obs         # relative location of the observer
