#!/usr/bin/env python

import roslib
import rospy
import tf
from gazebo_msgs.msg import LinkStates

first_read_flag = True
pos_x = pos_y = pos_z = 0
ori_x = ori_y = ori_z = ori_w = 0

def command_calback(msg):
        global first_read_flag
	if (first_read_flag == True) :
	      global pos_x , pos_y , pos_z, ori_x , ori_y , ori_z , ori_w 
	      names = msg.name
	      if 'Sahar::IPON::link' in names:
		      index = names.index('Sahar::IPON::link')
		      
		      pos_x = msg.pose[index].position.x
		      pos_y = msg.pose[index].position.y
		      pos_z = msg.pose[index].position.z
		      
		      ori_x = 0; msg.pose[index].orientation.x
		      ori_y = 0; msg.pose[index].orientation.y
		      ori_z = 0; msg.pose[index].orientation.z
		      ori_w = 1; msg.pose[index].orientation.w
		      		    
		      first_read_flag = False
	      else:
		      print 'Sahar::IPON::link dosn\'t exist in the Gazebo'	
		
	br = tf.TransformBroadcaster()
	br.sendTransform( (pos_x, pos_y, pos_z),
			  (ori_x, ori_y ,ori_z, ori_w),
			  rospy.Time.now(),
                          "WORLD",
                          "world_gazebo")	
		

rospy.init_node('world_gazebo_to_WORLD_TF_pub')

def main():
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		msg = rospy.wait_for_message("/gazebo/link_states", LinkStates)
		command_calback(msg)
		rate.sleep()


main()


