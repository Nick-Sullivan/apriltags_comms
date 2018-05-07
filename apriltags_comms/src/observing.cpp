#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include "std_msgs/Int32.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "visualization_msgs/Marker.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <apriltags2_ros/AprilTagDetection.h>
#include "apriltags_comms/AprilTagsCommsMsg.h"
#include <tf/tf.h>

#include <pose_cov_ops/pose_cov_ops.h>

/* This node subscribes to a /tag_detections topic (producing tag detections 
 * relative to a cameras optical frame). It then converts each pose
 * (cam->tag) into estimations of where the tag is relative to the origin
 * (map->tag) and where the tag is relative to the observing robot (base->tag).
 * These are then published for use by a receiving node.
 * 
 *  Written by Nick Sullivan, The University of Adelaide. */

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::map;

using geometry_msgs::PoseWithCovariance;          //pc
using geometry_msgs::PoseWithCovarianceStamped;   //pcs
using geometry_msgs::TransformStamped;            //ts
using nav_msgs::Odometry;                         //o
using apriltags_comms::AprilTagsCommsMsg;
using apriltags2_ros::AprilTagDetection;
using apriltags2_ros::AprilTagDetectionArray;


string         map_frame;            //world frame, same for all robots
string         base_frame;           //robot frame, moves with each robot
string         tag_frame_prefix;     //frame of visual tags (minus the num)
int            robot_id;             //ID of this robot
vector<double> preset_cov;           //visual detection uncertainty

Odometry o_map2base;                 //map->base_link transform.

ros::Subscriber sub_map2base;
ros::Subscriber sub_apriltags;
ros::Publisher  pub_apriltags_comms;

tf2_ros::Buffer tfbuffer;            //for transformation lookup

/**************************************************
 * Helper functions
 **************************************************/

// Publish a transformation to the transformation tree.
/*void publishTF(Odometry o){
  cout << "PUBLISHING TF" << endl;
  static tf2_ros::TransformBroadcaster br;
  TransformStamped ts;
  ts.header         = o.header;
  ts.child_frame_id = o.child_frame_id;
  ts.transform.translation.x = o.pose.pose.position.x;
  ts.transform.translation.y = o.pose.pose.position.y;
  ts.transform.translation.z = o.pose.pose.position.z;
  ts.transform.rotation      = o.pose.pose.orientation;
  br.sendTransform(ts);
}*/
 
// Combines two poses with covariance. Must be in the form
// A->B->C, where p1 is A->B and p2 is B->C.
PoseWithCovariance compose(PoseWithCovariance p1,
                           PoseWithCovariance p2){
  PoseWithCovariance out;
  pose_cov_ops::compose(p1,p2,out);  // out = p1 + p2
  return out;
}

// Looks up a transform from frame1 to frame2 using tf2. Returns whether the lookup
// was successful.
bool lookupTransform(TransformStamped &output, string frame1, string frame2, ros::Time t){
  try{
    cout << "Trying to get from frame: " << frame1 << " to " << frame2 << endl;                           
    if (!tfbuffer.canTransform(frame1, frame2, t) ){
      ROS_WARN("Unable to obtain a TF in observing.cpp");
      return false;
    }
    output = tfbuffer.lookupTransform(frame1, frame2, t);                                         
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return false;
  }
  return true;
}

// Given tag IDs, returns the tag frame string.  e.g. [1,4,3]->"tag_1_3_4".
string getTagFrame(vector<int> tag_ids){
  std::sort( tag_ids.begin(), tag_ids.end() );
  std::ostringstream strs;
  strs << "tag";
  for( int i=0; i<tag_ids.size(); i++ ){
    strs << "_" << tag_ids[i];
  }
  return strs.str();
}

// Converts a TF message into a Pose message. May optionally enter the covariance
// from the preset_cov variable.
PoseWithCovarianceStamped createPoseCovFromTF( TransformStamped tf_msg, bool use_preset_cov ){
  PoseWithCovarianceStamped pcs;
  pcs.header                = tf_msg.header;  
  pcs.pose.pose.position.x  = tf_msg.transform.translation.x;
  pcs.pose.pose.position.y  = tf_msg.transform.translation.y;
  pcs.pose.pose.position.z  = tf_msg.transform.translation.z;
  pcs.pose.pose.orientation = tf_msg.transform.rotation;
  if( use_preset_cov ){
    for( int i=0; i<36; i++ ){
      pcs.pose.covariance[i] = preset_cov[i];
    }
  }
  return pcs;
}

// Given two TF frames, produces a PoseWithCovarianceStamped message from frame1 to frame2. 
// Optionally populates the covariance with 'preset_cov'.
bool getPoseCovFromTFTree( PoseWithCovarianceStamped& pcs, string frame1, string frame2, bool use_preset_cov  ){
  TransformStamped ts;
  if( !lookupTransform(ts, frame1, frame2, ros::Time(0)) ){
    ROS_WARN("Failed to look up transformation");
    return false;
  }
  pcs = createPoseCovFromTF( ts, use_preset_cov );
  return true;
}

// Given two TF frames, produces an Odometry message from frame1 to frame2. 
// Optionally populates the covariance with 'preset_cov'.
bool getOdomFromTFTree( Odometry& o, string frame1, string frame2, bool use_preset_cov  ){
  PoseWithCovarianceStamped pcs;
  if( !getPoseCovFromTFTree( pcs, frame1, frame2, use_preset_cov ) ){
    return false;
  }
  o.header         = pcs.header;
  o.pose           = pcs.pose;
  o.child_frame_id = frame2;
  return true;
}


/**************************************************
 * Publisher functions
 **************************************************/

// Publish the AprilTagsCommsMsg.
void publishInterRobot( string tag_frame, Odometry o_map2tag, Odometry o_base2tag ){
    AprilTagsCommsMsg msg;
    msg.sender_robot_id = robot_id;
    msg.tag_frame       = tag_frame;
    msg.map2tag         = o_map2tag;
    msg.base2tag        = o_base2tag;
    pub_apriltags_comms.publish(msg);
    cout << "obs: Published comms" << endl;
}

// Upon tag detection, produce the map2tag and base2tag odometries, then publishes.
void processTagDetection( AprilTagDetection detection ){
  cout << "obs: Received a tag detection. " << endl;
  string tag_frame, cam_frame;
  tag_frame = getTagFrame(detection.id);
  cam_frame = detection.pose.header.frame_id;
  
  PoseWithCovarianceStamped pcs_cam2tag = detection.pose;
  // TF lookup: base(obs)->cam
  PoseWithCovarianceStamped pcs_base2cam;
  if( !getPoseCovFromTFTree(pcs_base2cam, base_frame, cam_frame, false) ) return;
  // TF composition: base(obs)->tag = base(obs)->cam + cam->tag
  Odometry o_base2tag;
  o_base2tag.header         = pcs_base2cam.header;
  o_base2tag.pose           = compose(pcs_base2cam.pose, pcs_cam2tag.pose);
  o_base2tag.child_frame_id = tag_frame;
  // TF composition: map->tag = map->base(obs) + base(obs)->tag
  Odometry o_map2tag;
  o_map2tag.pose           = compose(o_map2base.pose, o_base2tag.pose);
  o_map2tag.header         = o_map2base.header;
  o_map2tag.header.stamp   = o_base2tag.header.stamp;
  o_map2tag.child_frame_id = tag_frame;
  // Publish the message.
  publishInterRobot( tag_frame, o_map2tag, o_base2tag );
  
  cout << "obs: base2cam: (" << pcs_base2cam.pose.pose.position.x << " , " 
       << pcs_base2cam.pose.pose.position.y << " , " 
       << pcs_base2cam.pose.pose.position.z << ")" << endl;
  cout << "obs: cam2tag: (" << pcs_cam2tag.pose.pose.position.x << " , " 
       << pcs_cam2tag.pose.pose.position.y << " , " 
       << pcs_cam2tag.pose.pose.position.z << ")" << endl;     
       
  cout << "obs: map2base: (" << o_map2base.pose.pose.position.x << " , " 
       << o_map2base.pose.pose.position.y << " , " 
       << o_map2base.pose.pose.position.z << ")" << endl;
  cout << "obs: base2tag: (" << o_base2tag.pose.pose.position.x << " , " 
       << o_base2tag.pose.pose.position.y << " , " 
       << o_base2tag.pose.pose.position.z << ")" << endl;
  cout << "obs: map2tag: (" << o_map2tag.pose.pose.position.x << " , " 
       << o_map2tag.pose.pose.position.y << " , " 
       << o_map2tag.pose.pose.position.z << ")" << endl;
  
}


/**************************************************
 * Subscriber functions
 **************************************************/

// Keeps track of the most recent map->base transformation.
void callbackMap2Base(const Odometry::ConstPtr& msg) {
  o_map2base = *msg;
}

// Callback for apriltags. If pre-checks pass, fowards to 'processTagDetection'.
void callbackTagDetection(const AprilTagDetectionArray& apriltag_msg){
  for(vector<AprilTagDetection>::const_iterator it = apriltag_msg.detections.begin(); 
        it != apriltag_msg.detections.end(); 
        ++it){
	  AprilTagDetection detection = *it;
    if( detection.pose.pose.pose.position.z < 0 ) continue;
    processTagDetection(detection);
  }
}
 
/**************************************************
 * Initialising functions
 **************************************************/
void loadSubs(ros::NodeHandle n){
  sub_apriltags = n.subscribe("tag_detections", 100, callbackTagDetection);
  sub_map2base  = n.subscribe("map_to_base", 100, callbackMap2Base);                                               
}

void loadPubs(ros::NodeHandle n){
  pub_apriltags_comms = n.advertise<AprilTagsCommsMsg>("/apriltags_comms", 100);
}

// Loads parameters from the launch file. Uses default values if any values are
// missing.
void loadParams(ros::NodeHandle n_priv){
  // Set default parameters.
  string default_map_frame        = "map";
  string default_base_frame       = "base_link";
  string default_tag_frame_prefix = "ar_marker_";
  int    default_robot_id         = 0;
  bool   default_restamp_msgs     = false;
  double default_preset_cov[]     = {0.01,    0,    0,    0,    0,    0,
                                        0, 0.01,    0,    0,    0,    0,
                                        0,    0, 0.01,    0,    0,    0,
                                        0,    0,    0, 0.05,    0,    0,
                                        0,    0,    0,    0, 0.05,    0,
                                        0,    0,    0,    0,    0, 0.05 };
  // Check parameter server to override defaults.
  XmlRpc::XmlRpcValue v;
  n_priv.param(        "robot_id",         robot_id,         default_robot_id);
  n_priv.param(       "map_frame",        map_frame,        default_map_frame);
  n_priv.param(      "base_frame",       base_frame,       default_base_frame);
  n_priv.param("tag_frame_prefix", tag_frame_prefix, default_tag_frame_prefix);
  if( n_priv.getParam("preset_cov", v) ){       // x y z, roll pitch yaw
    ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i=0; i < v.size(); i++)  {
      if( v[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ) {
        preset_cov.push_back(v[i]);
      }
      if( v[i].getType() == XmlRpc::XmlRpcValue::TypeInt ) {
        int d = v[i];
        preset_cov.push_back(d);
      }
    }
  } else {
    preset_cov.assign(default_preset_cov, default_preset_cov + 
                      sizeof(default_preset_cov) / sizeof(default_preset_cov[0]) );
  }
}

/**************************************************
 * Main
 **************************************************/
int main(int argc, char** argv){
  ros::init(argc, argv, "observer");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~");
  tf2_ros::TransformListener tflistener(tfbuffer);   //must stay persistent
  cout << "Starting!" << endl;
  loadParams(n_priv);
  loadPubs(n);
  loadSubs(n);
  ros::spin();
  return 0;
};

