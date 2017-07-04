#!/usr/bin/env python

import rospy
import tf
import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Vector3


loc_RPY_pub = rospy.Publisher('/LOC_RolPithchYow', Vector3 , queue_size=5)

def command_calback(msg):
		quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		
		RPY = Vector3(roll,pitch,yaw)
		
		loc_RPY_pub.publish(RPY)


rospy.init_node('LOC_RolPithchYow_pub')

def main():
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		msg = rospy.wait_for_message("/LOC/Pose", PoseWithCovarianceStamped)
		command_calback(msg)
		rate.sleep()

main()

