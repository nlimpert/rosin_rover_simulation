#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "geometry_msgs/Twist.h"	

#include <sstream>


bool publish = false;
double one_degree=0.0174532862792735;

ros::Publisher pub;
ros::Subscriber sub;

double steer_angle;
double steer_vel;
double speed_vel;
double accel;
int invert;


ackermann_msgs::AckermannDriveStamped ackermann_msg;

void ackermann_cmd_Callback(const geometry_msgs::Twist::ConstPtr& msg);


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{



  ros::init(argc, argv, "ackermann_cmd_wrapper");

  ros::NodeHandle n;


n.param<double>("steering_angle" , steer_angle , .0);
n.param<double>("steering_velocity" , steer_vel , .0);
n.param<double>("max_speed" , speed_vel , 1.);
n.param<double>("acceleration" , accel , 0.);
n.param<int>("invert_steering", invert , 1);

steer_angle *= one_degree;
steer_vel *= one_degree;

 pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd", 1000);

 sub = n.subscribe("cmd_vel", 1000, ackermann_cmd_Callback);


  ros::Rate loop_rate(100);

	ROS_INFO("**** ACKERMANN WRAPPER Initialized ****");
	ROS_INFO("steering_angle: %lf", steer_angle/one_degree);	
	ROS_INFO("steering_velocity: %lf", steer_vel/one_degree);	
	ROS_INFO("max_speed: %lf", speed_vel);	
	ROS_INFO("acceleration: %lf", accel);	
	ROS_INFO("invert_steering: %d", invert);	
	ROS_INFO("***************************************");

  unsigned int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

void ackermann_cmd_Callback(const geometry_msgs::Twist::ConstPtr& msg)
{

	//ROS_INFO("GOT TWIST_MESSAGE");

	ackermann_msg.drive.steering_angle = msg->angular.z*invert*steer_angle; 
	

	ackermann_msg.drive.steering_angle_velocity = steer_vel; 
	ackermann_msg.drive.speed = msg->linear.x*speed_vel;
	ackermann_msg.drive.acceleration = accel;
	
	pub.publish(ackermann_msg);

}

