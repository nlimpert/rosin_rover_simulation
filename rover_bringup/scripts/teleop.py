#!/usr/bin/env python
###################  import  ###########################
import rospy 
from sensor_msgs.msg import Joy		#joystick message type
from geometry_msgs.msg import Twist	#rover message type

############# creation of objects  #####################
move=Twist()

#############  node initialization  ####################
rospy.init_node('teleop', anonymous=True)

##### initialize values ####
move.linear.x = 0	
move.angular.z = 0

############ definitions of functions ##################
def callback(data):
	global move			

	move.linear.x=data.axes[1]
	move.angular.z=data.axes[2]

#### definition of publisher/subscriber and services ###
rospy.Subscriber("joy", Joy, callback)		#Subscriber from Joystick
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)	#Publisher to Rover

############# main program #############################
rate = rospy.Rate(10)

#--------------endless loop till shut down -------------#
while not rospy.is_shutdown():
	pub.publish(move)		#publish					
	rate.sleep()
