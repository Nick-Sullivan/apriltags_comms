#!/usr/bin/env python
import rospy
import tf2_ros
from std_msgs.msg        import String
from geometry_msgs.msg   import PoseWithCovarianceStamped
from apriltags_comms.msg import AprilTagsCommsMsg
from apriltags2_ros.msg  import AprilTagDetection
from apriltags2_ros.msg  import AprilTagDetectionArray


def compose(p1, p2):
  pose_cov_ops::compose(p1,p2,out);  # out = p1 + p2
  return out;


class observer:
  
  def __init__(self):
    self.subs()
    self.pubs()
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
  
  def subs(self):
    rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tagDetectionCallback)
    
  def pubs(self):
    pub = rospy.Publisher('apriltags_comms', String, queue_size=10)
  
  # Returns a TransformStamped message, or False if no TF could be found.
  def lookupTransform(self, frame_from, frame_to):
    try:
      trans = self.tfBuffer.lookup_transform(frame_from, frame_to, rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      print "Could not find transformation from " + frame_from + " to " + frame_to
      return False
    return trans
  
  # Given a valid AprilTagDetection from cam->tag, this processes it to produce 
  # output odometries from map->tag and base->tag.
  def processTagDetection(self, detection):
    print "Processing a detection"
    # Get a TF from base->cam from the TF tree.
    tf_base2cam = self.lookupTransform('base_link', 'cam_opt')
    if not tf_base2cam:
      return   
    print tf_base2cam
    
    # base(obs)->tag = base(obs)->cam + cam->tag
    #o_base2tag = compose(pcs_baselink_to_cam.pose, pcs_cam_to_tag.pose);

  # Called periodically with AprilTags messages. Performs pre-checks, then passes
  # detections to 'processTagDetection' if detection is valid.
  def tagDetectionCallback(self, detection_array_msg):
    for detection in detection_array_msg.detections:
      id_array    = detection.id
      pcs_cam2tag = detection.pose
      if pcs_cam2tag.pose.pose.position.z < 0:
        continue
      self.processTagDetection(detection)

    
if __name__ == '__main__':
  obs = observer()
  rospy.init_node('observer', anonymous=True)
  rospy.spin()
