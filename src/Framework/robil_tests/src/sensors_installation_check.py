#!/usr/bin/env python

import rospy
import tf
import math
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates


sensors = ['IPON','IBEO']

def command_calback(msg):
	names = msg.name
	print '\n \n \n'
	for sensor in sensors:
	    s = 'Sahar::'+sensor+'::link'
	    if s in names:	   
		index_sensor = names.index(s) 
		index_body = names.index("Sahar::body") 

		t = tf.Transformer(True, rospy.Duration(10.0))

		sensor_tf = geometry_msgs.msg.TransformStamped()
		sensor_tf.header.frame_id = 'global'
		sensor_tf.child_frame_id = 'sensor'
		sensor_tf.transform.translation.x = msg.pose[index_sensor].position.x
		sensor_tf.transform.translation.y = msg.pose[index_sensor].position.y
		sensor_tf.transform.translation.z = msg.pose[index_sensor].position.z
		sensor_tf.transform.rotation.x = msg.pose[index_sensor].orientation.x
		sensor_tf.transform.rotation.y = msg.pose[index_sensor].orientation.y
		sensor_tf.transform.rotation.z = msg.pose[index_sensor].orientation.z
		sensor_tf.transform.rotation.w = msg.pose[index_sensor].orientation.w
		t.setTransform(sensor_tf)

		body_tf = geometry_msgs.msg.TransformStamped()
		body_tf.header.frame_id = 'global'
		body_tf.child_frame_id = 'body'
		body_tf.transform.translation.x = msg.pose[index_body].position.x
		body_tf.transform.translation.y = msg.pose[index_body].position.y
		body_tf.transform.translation.z = msg.pose[index_body].position.z
		body_tf.transform.rotation.x = msg.pose[index_body].orientation.x
		body_tf.transform.rotation.y = msg.pose[index_body].orientation.y
		body_tf.transform.rotation.z = msg.pose[index_body].orientation.z
		body_tf.transform.rotation.w = msg.pose[index_body].orientation.w
		t.setTransform(body_tf)

		#print t.getFrameStrings()

                print sensor + ' Position '  
		
		position, quaternion = t.lookupTransform('global', 'sensor', rospy.Time(0))
		euler = tf.transformations.euler_from_quaternion(quaternion)
		
		print 'GLOBAL_frame :   x =  %6.4f ' %position[0]  + '    y =  %6.4f ' %position[1]  + '    z =  %6.4f ' %position[2]  , '  roll =  %6.4f ' %euler[0]  + '  pitch =  %6.4f ' %euler[1]  + '  yaw =  %6.4f ' %euler[2]  

		position, quaternion = t.lookupTransform('body', 'sensor', rospy.Time(0))
		euler = tf.transformations.euler_from_quaternion(quaternion)

		print 'body_frame :   x =  %6.4f ' %position[0]  + '    y =  %6.4f ' %position[1]  + '    z =  %6.4f ' %position[2]  , '  roll =  %6.4f ' %euler[0]  + '  pitch =  %6.4f ' %euler[1]  + '  yaw =  %6.4f ' %euler[2]  
		print '\n'
	    else:
		print sensor + ' dosn\'t exist in the Gazebo'

rospy.init_node('print_sensors_position')

def main():
		msg = rospy.wait_for_message("/gazebo/link_states", ModelStates)
		command_calback(msg)

main()




