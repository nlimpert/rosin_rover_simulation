#include <marker_localization.h>


void sending(tf::Transform trans)
{

    if(two_D)
    {
      //Setting Z component to zero
      tf::Vector3 origin;
      //trans.getOrigin().setZ()
      origin = trans.getOrigin();
      origin.setZ(tfScalar(0.0));
      trans.setOrigin(origin);

      //Setting roll and pitch to zero
      tfScalar roll, pitch, yaw;
      trans.getBasis().getRPY(roll, pitch, yaw);
      roll = 0.0;
      pitch = 0.0;
      tf::Quaternion quat;
      quat.setRPY(roll,pitch,yaw);
      trans.setRotation(quat);
    }

    //broadcast transform
    if(broadcast_tf)
        tf_broadcaster->sendTransform(tf::StampedTransform(trans, ros::Time::now(), output_frame, base_frame));

    if(publish_pose)
    {
        //generating PoseStamped Msg out of transform
        geometry_msgs::Transform trans_tmp;
        geometry_msgs::PoseStamped pose_robot;
        tf::transformTFToMsg(trans,trans_tmp);
        pose_robot.pose.orientation = trans_tmp.rotation;
        pose_robot.pose.position.x = trans_tmp.translation.x;
        pose_robot.pose.position.y = trans_tmp.translation.y;
        pose_robot.pose.position.z = trans_tmp.translation.z;
        pose_robot.header.stamp = ros::Time::now();
        pose_robot.header.frame_id = output_frame;
        //publish Pose message
        pub_pose.publish(pose_robot);
    }

}



void chatterCallback(const ar_track_alvar_msgs::AlvarMarkers& arPoseMarkers_) {

    for(size_t i=0; i < arPoseMarkers_.markers.size(); i++) {

        //Get the ID for the specific marker
        std::stringstream out;
        out << "ar_marker_" << arPoseMarkers_.markers[i].id;
        std::string marker_frame = out.str();

        //setup the needed transforms
        tf::Transform t_cam_to_marker;
        tf::Transform t_marker_to_cam;
        tf::Transform t_marker_to_base;
        tf::StampedTransform T_cam_to_base;

        tf::StampedTransform T_out_to_marker;
        tf::Transform t_out_to_base;

        // ### 1. Get the transform from the marker to the camera ###
        // we are not considering the timestamps, that's not the perfect clean way, but for our specification it's sufficient!
        // let's start to get the transfrom cam -> marker by using the published pose of ar_track_alvar
        tf::poseMsgToTF(arPoseMarkers_.markers[i].pose.pose, t_cam_to_marker);
        // now get marker -> cam by using the inverse function
        t_marker_to_cam = t_cam_to_marker.inverse();

        // ### 2. Get the transform from marker to the robot's base frame
        try{
          //Get transform camera frame to robots base frame
          tf_listener->lookupTransform(camera_frame, base_frame,ros::Time(0), T_cam_to_base);

          //calculate the transform from the marker to the robot's base_frame
          t_marker_to_base = t_marker_to_cam * T_cam_to_base;
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }

        // ### 3. Get the transform from output frame to base frame of the robot
        // Note: The marker must have a transform available in relation to the output frame!

        try{
          // Get the transform output -> marker
          tf_listener->lookupTransform(output_frame, marker_frame,ros::Time(0), T_out_to_marker);
          //calculate the transform from output to base of the robot
          t_out_to_base = T_out_to_marker * t_marker_to_base;

        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }

        //Send the results to the ROS System
        sending(t_out_to_base);
    }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle n("~");

  //setup publisher and subscriber
  sub = n.subscribe("/ar_pose_marker", 1, chatterCallback);
  pub_pose = n.advertise<geometry_msgs::PoseStamped>("pose_rover",1);

  //setup parameters
  n.param<std::string>("base_frame", base_frame, "base_footprint");
  n.param<std::string>("camera_frame", camera_frame, "camera_link");
  n.param<std::string>("output_frame", output_frame, "map");

  n.param("broadcast_tf", broadcast_tf, true);
  n.param("publish_pose", publish_pose, true);
  n.param("two_D", two_D, true);

  //setup tf listener and broadcaster
  tf_listener = new tf::TransformListener(n);
  tf_broadcaster = new tf::TransformBroadcaster();

  //spin to the win
  ros::spin();
  return 0;
}
