#!/usr/bin/env python
import rospy
import tf2_ros
import tf.transformations
from math                import cos, sin
from std_msgs.msg        import String
from nav_msgs.msg        import Odometry
from geometry_msgs.msg   import PoseWithCovarianceStamped
from apriltags_comms.msg import AprilTagsCommsMsg
from apriltags2_ros.msg  import AprilTagDetection
from apriltags2_ros.msg  import AprilTagDetectionArray



class observer:
  
  def __init__(self):
    self.subs()
    self.pubs()
    self.params()
    #self.tfBuffer = tf2_ros.Buffer()
    #self.listener = tf2_ros.TransformListener(self.tfBuffer)
  
  def subs(self):
    rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tagDetectionCallback)
    rospy.Subscriber("map_to_base",    Odometry,               self.mapToBaseCallback)
    
  def pubs(self):
    self.pub = rospy.Publisher('/apriltags_comms', AprilTagsCommsMsg, queue_size=10)
  
  def params(self):
    self.robot_id = rospy.get_param('~robot_id', 1)
    
  # Returns a TransformStamped message, or False if no TF could be found.
  #def lookupTransform(self, frame_from, frame_to):
  #  try:
  #    trans = self.tfBuffer.lookup_transform(frame_from, frame_to, rospy.Time())
  #  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
  #    print "Could not find transformation from " + frame_from + " to " + frame_to
  #    return False
  #  return trans
  
  def getTagFrame( self, tag_id_vec ):
    name = "tag"
    tag_id_vec = sorted( tag_id_vec )
    for i in tag_id_vec:
      name = name + "_" + str(i)
    return name

  
  def processTagDetection(self, header, name, detection):
    print "Processing a detection"
    # 1) Detection is from camera->tag.
    # Convert interrobot pose (Z forward, X right) to ROS (X foward, Y left)
    pose = detection.pose.pose.pose
    dx =  pose.position.z
    dy = -pose.position.x
    quat = (pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quat)
    dyaw = euler[2]
    # Rotate detection according to the camera that observed it.
    pi  = 3.14159265
    yaw = 0;
    #if   header.frame_id == 'left_camera_optical':
    #  yaw +=  pi/2
    #elif header.frame_id == 'right_camera_optical':
    #  yaw += -pi/2
    #elif header.frame_id == 'forward_camera_optical':
    #  yaw +=  0
    #elif header.frame_id == 'backward_camera_optical':
    #  yaw +=  pi
    #else:
    #  print('Incorrect frame id')
    #  print( header.frame_id )
    if   "left_camera_optical" in header.frame_id:
      yaw +=  pi/2
    elif "right_camera_optical" in header.frame_id:
      yaw += -pi/2
    elif "forward_camera_optical" in header.frame_id or "front_camera_optical" in header.frame_id:
      yaw +=  0
    elif "backward_camera_optical" in header.frame_id:
      yaw +=  pi
    else:
      print('Incorrect frame id')
      print( header.frame_id )
      
    dx2   = dx* cos(-yaw) + dy*sin(-yaw)
    dy2   = dx*-sin(-yaw) + dy*cos(-yaw)
    dyaw2 = dyaw + yaw
    dyaw2 = dyaw2 + pi  #they were all calibrated backwards?
    
    # 2) Detection is from base_link -> tag
    # Rotate detection according to the observers location.
    x   = self.x + dx2* cos(-self.yaw) + dy2*sin(-self.yaw)
    y   = self.y + dx2*-sin(-self.yaw) + dy2*cos(-self.yaw)
    yaw = self.yaw + dyaw2
    x_cov   = self.x_cov   + 100
    y_cov   = self.y_cov   + 100
    yaw_cov = self.yaw_cov + 10
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    
    map2tag = self.map2obs
    map2tag.pose.pose.position.x    = x
    map2tag.pose.pose.position.y    = y
    map2tag.pose.pose.position.z    = 0
    map2tag.pose.pose.orientation.x = quat[0]
    map2tag.pose.pose.orientation.y = quat[1]
    map2tag.pose.pose.orientation.z = quat[2]
    map2tag.pose.pose.orientation.w = quat[3]
    map2tag.pose.covariance = (x_cov,     0,   0,   0,   0,       0,
                                   0, y_cov,   0,   0,   0,       0,
                                   0,     0, 1e3,   0,   0,       0,
                                   0,     0,   0, 1e3,   0,       0,
                                   0,     0,   0,   0, 1e3,       0,
                                   0,     0,   0,   0,   0, yaw_cov)
    map2tag.twist.twist.linear.x  = 0
    map2tag.twist.twist.linear.y  = 0
    map2tag.twist.twist.linear.z  = 0
    map2tag.twist.twist.angular.x = 0
    map2tag.twist.twist.angular.y = 0
    map2tag.twist.twist.angular.z = 0
    map2tag.twist.covariance      = (1e3,   0,   0,   0,   0,   0,
                                       0, 1e3,   0,   0,   0,   0,
                                       0,   0, 1e3,   0,   0,   0,
                                       0,   0,   0, 1e3,   0,   0,
                                       0,   0,   0,   0, 1e3,   0,
                                       0,   0,   0,   0,   0, 1e3)
    # 3) Detection is from map -> tag
    msg = AprilTagsCommsMsg()
    msg.obs_robot_id = self.robot_id
    msg.tag_frame    = name
    msg.map2obs      = self.map2obs
    msg.map2tag      = map2tag
    
    self.pub.publish(msg)
    

  # Called periodically with AprilTags messages. Performs pre-checks, then passes
  # detections to 'processTagDetection' if detection is valid.
  def tagDetectionCallback(self, detection_array_msg):
    header = detection_array_msg.header
    for detection in detection_array_msg.detections:
      pcs_cam2tag = detection.pose
      name        = self.getTagFrame( detection.id )
      if pcs_cam2tag.pose.pose.position.z < 0:
        continue
      self.processTagDetection(header, name, detection)

  # Stores x, y, and yaw from the location message.
  def mapToBaseCallback(self, msg):
    self.x   = msg.pose.pose.position.x
    self.y   = msg.pose.pose.position.y
    quat     = (msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
    euler    = tf.transformations.euler_from_quaternion(quat)
    self.yaw = euler[2]
    self.x_cov   = msg.pose.covariance[0]
    self.y_cov   = msg.pose.covariance[7]
    self.yaw_cov = msg.pose.covariance[35]
    self.map2obs = msg
    
if __name__ == '__main__':
  rospy.init_node('observer', anonymous=True)
  obs = observer()
  rospy.spin()
