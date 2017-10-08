#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <std_msgs/String.h>


ros::Subscriber sub;
ros::Publisher pub_pose;


std::string output_frame, base_frame, camera_frame;
bool broadcast_tf, publish_pose, two_D;

tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;
