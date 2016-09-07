#!/usr/bin/env python

import rospy
import tf
import math
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped

velocity_linear_command_pub = rospy.Publisher('/RQT_paltform_velocity_monitor/linear', Float32 , queue_size=5)
velocity_angular_command_pub = rospy.Publisher('/RQT_paltform_velocity_monitor/angular', Float32 , queue_size=5)
yaw = 0

def callback_yaw(msg):
	quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
#	roll = euler[0]
#	pitch = euler[1]
	yaw = euler[2]

def command_calback(msg):
	names = msg.name
	if 'Sahar' in names:
		index = names.index('Sahar')
		velocityToPub = Float32()
		squareOfX = math.pow(msg.twist[index].linear.x,2)
		squareOfY = math.pow(msg.twist[index].linear.y,2)
		V_normal = math.sqrt(squareOfX + squareOfY)
		velocityToPub.data = 0
		if(V_normal > 0.01):
			x_normal = msg.twist[index].linear.x/V_normal
			y_normal = msg.twist[index].linear.y/V_normal
			theta = math.atan2(y_normal, x_normal)
			if(abs(theta - yaw) - (math.pi/2) > 0.01):
				velocityToPub.data = -V_normal
			else:
				velocityToPub.data = V_normal
		velocity_linear_command_pub.publish(velocityToPub)
		velocity_angular_command_pub.publish(msg.twist[index].angular.z)
	else:
		print 'Sahar dosn\'t exist in the Gazebo'

rospy.init_node('velocity_command_pub')

def main():
	rate = rospy.Rate(100)
	rospy.Subscriber("/LOC/Pose", PoseWithCovarianceStamped, callback_yaw)
	while not rospy.is_shutdown():
		msg = rospy.wait_for_message("/gazebo/model_states", ModelStates)
		command_calback(msg)
		rate.sleep()

main()

