#!/usr/bin/env python
###################  import  ###########################
import rospy 
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

############# creation of objects  #####################
distance=Twist()	

#############  node initialization  ####################
rospy.init_node('joystick', anonymous=True)	#init_node

############# creation variables  #######################
flag = True 		#flag is set when obstacle detected

############ definitions of functions ##################
def clbk_obstacle(data):
	
	global flag
	check = data.range	#get value of distance
	if check <= 0.5:	#obstacle at 0.5m
		flag = True
	else:
		flag = False

def clbk_drive(data):
	global distance

	distance.linear.x=data.axes[1]
	distance.angular.z=data.axes[2]
	
#### definition of publisher/subscriber and services ###
rospy.Subscriber("joy", Joy, clbk_drive)			#subscribe Gamepad
rospy.Subscriber("range", Range, clbk_obstacle)			#subscribe rover sensor
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)		#publish to rover
	
############# main program #############################
rate = rospy.Rate(10)

#--------------endless loop till shut down -------------#	
while not rospy.is_shutdown():
		
	if flag == True:
		distance.linear.x = 0
	else:
		pass
	pub.publish(distance)	
	rate.sleep()
