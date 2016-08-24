#!/usr/bin/env python

import rospy
import math
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32

velocity_linear_command_pub = rospy.Publisher('/RQT_paltform_velocity_monitor/linear', Float32 , queue_size=5)
velocity_angular_command_pub = rospy.Publisher('/RQT_paltform_velocity_monitor/angular', Float32 , queue_size=5)
def command_calback(msg):
	names = msg.name
	if 'Sahar' in names:
		index = names.index('Sahar')
		velocityToPub = Float32()
		squareOfX = math.pow(msg.twist[index].linear.x,2)
		squareOfY = math.pow(msg.twist[index].linear.y,2)
		velocityToPub.data = math.sqrt(squareOfX + squareOfY)
		velocity_linear_command_pub.publish(velocityToPub)
		velocity_angular_command_pub.publish(msg.twist[index].angular.z)
	else:
		print 'Sahar dosn\'t exist in the Gazebo'

rospy.init_node('velocity_command_pub')

def main():
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		msg = rospy.wait_for_message("/gazebo/model_states", ModelStates)
		command_calback(msg)
		rate.sleep()

main()

