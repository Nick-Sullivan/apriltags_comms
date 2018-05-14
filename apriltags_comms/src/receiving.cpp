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
bool           restamp_msgs;         //received messages are timestamped with the time it was received
bool           publish_map2base;     //publish the tf from map to base
map<string,string> tag_frame_remap;  //

Odometry o_map2base;                 //map->base_link transform.

ros::Subscriber sub_map2base;
ros::Subscriber sub_apriltags_comms;
ros::Publisher  pub_tracked_pose;
ros::Publisher  pub_interrobot;

tf2_ros::Buffer tfbuffer;            //for transformation lookup

/**************************************************
 * Helper functions
 **************************************************/

// Publish a transformation to the transformation tree.
void publishTF(Odometry o){
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
}
 
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

// If a tag frame has been marked for renaming, this will return the new name.
// Otherwise, it will return the default name.
string lookupTagFrameRename(string tag_frame){
  string new_name;
  try {
    new_name = tag_frame_remap.at(tag_frame);
  } catch( const std::out_of_range& e ){
    return tag_frame;
  }
  return new_name;
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
/*void publishInterRobot( string tag_frame, Odometry o_map2tag, Odometry o_base2tag ){
    AprilTagsCommsMsg msg;
    msg.sender_robot_id = robot_id;
    msg.tag_frame       = tag_frame;
    msg.map2tag         = o_map2tag;
    msg.base2tag        = o_base2tag;
    pub_apriltags_comms.publish(msg);
}*/

// Upon tag detection, produce the map2tag and base2tag odometries, then publishes.
void processAprilTagsComms( AprilTagsCommsMsg comms_msg ){
  cout << "rec" << robot_id << ": Received apriltags comms. " << endl;
  string tag_frame;
  tag_frame = lookupTagFrameRename(comms_msg.tag_frame);
  cout << "tag_frame: " << tag_frame << endl;
  
  Odometry o_map2tag, o_base2tag;
  o_map2tag  = comms_msg.map2tag;
  o_base2tag = comms_msg.base2tag;
  // TF lookup: tag->base(rec)
  PoseWithCovarianceStamped pcs_tag2base;
  if( !getPoseCovFromTFTree( pcs_tag2base, tag_frame, base_frame, false ) ){
     cout << "rec" << robot_id << ": Couldn't find posecov" << endl;
     return;
   }
  // TF composition: map->base(rec) = map->tag + tag->base(rec)
  Odometry o_map2base;
  o_map2base.pose           = compose(o_map2tag.pose, pcs_tag2base.pose);
  o_map2base.header         = o_map2tag.header;
  o_map2base.child_frame_id = base_frame;
  // Publish odometry.
  pub_tracked_pose.publish(o_map2base);
  if( publish_map2base ){
    publishTF(o_map2base);
  }
  
  // TF composition: base(obs)->base(rec) = base(obs)->tag + tag->base(rec)
  std::ostringstream strs, strs2;
  strs << o_base2tag.header.frame_id << comms_msg.sender_robot_id;
  strs2 << base_frame << robot_id;
  Odometry o_base2base;
  o_base2base.pose            = compose(o_base2tag.pose, pcs_tag2base.pose);
  o_base2base.header          = o_base2tag.header;
  o_base2base.header.frame_id = strs.str();
  o_base2base.child_frame_id  = strs2.str();
  // Publish odometry.
  pub_interrobot.publish(o_base2base);
  
  cout << "rec" << robot_id << ": odometry published" << endl;
  
  cout << "rec" << robot_id << ": map2tag: (" << o_map2tag.pose.pose.position.x << " , " 
       << o_map2tag.pose.pose.position.y << " , " 
       << o_map2tag.pose.pose.position.z << ")" << endl;
  cout << "rec" << robot_id << ": tag2base: (" << pcs_tag2base.pose.pose.position.x << " , " 
       << pcs_tag2base.pose.pose.position.y << " , " 
       << pcs_tag2base.pose.pose.position.z << ")" << endl;     
  cout << "rec" << robot_id << ": map2base: (" << o_map2base.pose.pose.position.x << " , " 
       << o_map2base.pose.pose.position.y << " , " 
       << o_map2base.pose.pose.position.z << ")" << endl;
       
  
  /*Odometry o_map_to_reclink, o_obslink_to_reclink, o_map_to_obslink;

  // obs->rec = obs->tag + tag->reclink
  o_obslink_to_reclink.pose           = compose(o_obslink_to_tag.pose, pcs_tag_to_reclink.pose); 
  o_obslink_to_reclink.header         = o_obslink_to_tag.header;
  o_obslink_to_reclink.child_frame_id = baselink_frame;
  // map->obslink = map->reclink + reclink->obslink
  pc_reclink_to_obslink = invertPoseCov(o_obslink_to_reclink.pose);
  o_map_to_obslink.pose           = compose(o_map_to_baselink.pose, pc_reclink_to_obslink); 
  o_map_to_obslink.header         = o_map_to_tag.header;
  o_map_to_obslink.child_frame_id = o_obslink_to_tag.header.frame_id; // check this.
  
                                                                                      
  if( restamp_msgs ){
    o_map_to_reclink.header.stamp     = ros::Time::now();
    o_obslink_to_reclink.header.stamp = ros::Time::now();
  }
  pub_tracked_pose.publish(o_map_to_reclink);
  // Possibly publish map->reclink TF.
  if( publish_tf ){
    publishTF(o_map_to_reclink);
  }
  
  // Send response.
  publishInterRobotResponse(sender_id, o_map_to_obslink, o_obslink_to_reclink);
  */
  
}


/**************************************************
 * Subscriber functions
 **************************************************/

// Keeps track of the most recent map->base transformation.
void callbackMap2Base(const Odometry::ConstPtr& msg) {
  o_map2base = *msg;
}

// Callback for communication from an observer node. If we have an estimate for
// where the observed tag is, we'll use that to improve our own pose estimate.
// Forwards to 'processAprilTagsComms'.
void callbackAprilTagsComms(const AprilTagsCommsMsg& comms_msg){
  if( comms_msg.sender_robot_id == robot_id ) return;
  cout << "rec" << robot_id << ": Received a callback" << endl;
  string tag_frame = lookupTagFrameRename(comms_msg.tag_frame);
  cout << "rec" << robot_id << ": tag_frame: " << tag_frame << endl;
  cout << "rec" << robot_id << ":Checking TF exists from " << tag_frame << " to " << base_frame << endl;
  ros::Time t, time_now;
  ros::Duration time_wiggle(1.0);
  time_now = ros::Time::now();
  if( time_now.sec > time_wiggle.sec ) {
    t = time_now - time_wiggle;
  } else {
    t = time_now;
  }
  cout << "t: " << t.sec << endl;
  if( !tfbuffer.canTransform(tag_frame, base_frame, t) ){
    cout << "rec" << robot_id << ": no TF exists" << endl;
    return;
  }
  processAprilTagsComms(comms_msg);
}
 
/**************************************************
 * Initialising functions
 **************************************************/
void loadSubs(ros::NodeHandle n){
  sub_apriltags_comms = n.subscribe("/apriltags_comms", 100, callbackAprilTagsComms);
  sub_map2base        = n.subscribe("map_to_base", 100, callbackMap2Base);                                               
}

void loadPubs(ros::NodeHandle n){
  pub_tracked_pose = n.advertise<Odometry>("odometry/tracked", 100);
  pub_interrobot   = n.advertise<Odometry>("odometry/robot2robot", 10);
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
  bool   default_publish_map2base = false;
  // Check parameter server to override defaults.
  XmlRpc::XmlRpcValue v;
  n_priv.param(        "robot_id",         robot_id,         default_robot_id);
  n_priv.param(       "map_frame",        map_frame,        default_map_frame);
  n_priv.param(      "base_frame",       base_frame,       default_base_frame);
  n_priv.param("tag_frame_prefix", tag_frame_prefix, default_tag_frame_prefix);
  n_priv.param(    "restamp_msgs",     restamp_msgs,     default_restamp_msgs);
  n_priv.param("publish_map2base", publish_map2base, default_publish_map2base);
  bool flip = true;
  string key;
  if( n_priv.getParam("tag_frame_remap", v) ){       // default_name, new_name
    ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i=0; i < v.size(); i++)  {
      if( v[i].getType() == XmlRpc::XmlRpcValue::TypeString ) {
        string input = static_cast<string>(v[i]);
        if( flip ){
          key = input;
          flip = false;
        } else {
          tag_frame_remap[key] = input;
          flip = true;
        }
      }
    }
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

