#!/usr/bin/env python

import roslib
import rospy
import tf
from gazebo_msgs.msg import LinkStates

def command_calback(msg):
	names = msg.name
	if 'Sahar::body' in names:
		index = names.index('Sahar::body')
		
		pos_x = msg.pose[index].position.x
		pos_y = msg.pose[index].position.y
		pos_z = msg.pose[index].position.z
		
		ori_x = msg.pose[index].orientation.x
		ori_y = msg.pose[index].orientation.y
		ori_z = msg.pose[index].orientation.z
		ori_w = msg.pose[index].orientation.w
		
		
		br = tf.TransformBroadcaster()
		br.sendTransform( (pos_x, pos_y, pos_z),
				  (ori_x, ori_y ,ori_z, ori_w),
				  rospy.Time.now(),
                                  "body",
                                  "WORLD")
	
	else:
		print 'Sahar::body dosn\'t exist in the Gazebo'


rospy.init_node('world_to_body_TF_pub')

def main():
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		msg = rospy.wait_for_message("/gazebo/link_states", LinkStates)
		command_calback(msg)
		rate.sleep()


main()


