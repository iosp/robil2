#!/usr/bin/env python

import rospy
import tf
import math
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped

velocity_linear_command_pub = rospy.Publisher('/RQT_paltform_velocity_monitor/linear', Float32 , queue_size=5)
velocity_angular_command_pub = rospy.Publisher('/RQT_paltform_velocity_monitor/angular', Float32 , queue_size=5)

def command_calback(msg):
	names = msg.name
	if 'Sahar' in names:
		index = names.index('Sahar')
		velocityToPub = Float32()
		AngularVel = Float32()
		
		AngularVel.data = msg.twist[index].angular.z
		
		squareOfX = math.pow(msg.twist[index].linear.x,2)
		squareOfY = math.pow(msg.twist[index].linear.y,2)
		V_size = math.sqrt(squareOfX + squareOfY) #+abs(AngularVel.data)*0.2

		quaternion = (msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z, msg.pose[index].orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
#		roll = euler[0]
#		pitch = euler[1]
		yaw = euler[2]
		

		velocityToPub.data = 0
		if(V_size > 0.01):
			x_normal = msg.twist[index].linear.x/V_size
			y_normal = msg.twist[index].linear.y/V_size
			theta = math.atan2(y_normal, x_normal)
			
			ang_diff = abs(theta - yaw)
			if  ( ang_diff - (float(3)/float(8)*math.pi)  < 0.01 or ang_diff>5):
				velocityToPub.data = V_size;
			elif( ang_diff - (float(5)/float(8)*math.pi)  < 0.01 ):
				velocityToPub.data = 0
			else:
				velocityToPub.data = -V_size



		velocity_linear_command_pub.publish(velocityToPub)
		velocity_angular_command_pub.publish(AngularVel)
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

