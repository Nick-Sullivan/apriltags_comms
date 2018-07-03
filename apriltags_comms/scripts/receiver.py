#!/usr/bin/env python
import rospy
from nav_msgs.msg        import Odometry
from apriltags_comms.msg import AprilTagsCommsMsg



class receiver:
  
  def __init__(self):
    self.subs()
    self.pubs()
    self.params()
  
  def subs(self):
    rospy.Subscriber("/apriltags_comms", AprilTagsCommsMsg, self.commsCallback)

  def pubs(self):
    self.pub = rospy.Publisher('odometry/tracked', Odometry, queue_size=10)
  
  def params(self):
    self.robot_id  = rospy.get_param('~robot_id', 1)
    self.box_frame = rospy.get_param('~box_frame', "tag_0_1_2_3")
    self.restamp   = rospy.get_param('~restamp', False)
  
  def processAprilTagsComms(self, comms):
    print "Processing a communication"
    map2tag = comms.map2tag
    if self.restamp:
      map2tag.header.stamp = rospy.get_rostime()
    self.pub.publish(map2tag)

  def commsCallback(self, comms_msg):
    if comms_msg.obs_robot_id == self.robot_id:
      return
    if comms_msg.tag_frame != self.box_frame:
      return
    self.processAprilTagsComms( comms_msg )

if __name__ == '__main__':
  rospy.init_node('observer', anonymous=True)
  obs = receiver()
  rospy.spin()
