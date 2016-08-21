#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32

WPD_command_pub_linear = rospy.Publisher('/RQT_WPD_command/linear', Float32 , queue_size=5)
WPD_command_pub_angular = rospy.Publisher('/RQT_WPD_command/angular', Float32 , queue_size=5)

def command_calback(msg):
   msgLinearToSnd = Float32()
   msgLinearToSnd.data = msg.twist.linear.x
   WPD_command_pub_linear.publish(msgLinearToSnd)
   msgAngularToSnd = Float32()
   msgAngularToSnd.data = msg.twist.angular.z   
   WPD_command_pub_angular.publish(msgAngularToSnd)
   rospy.sleep(0.01)

rospy.init_node('wpd_command_pub')
rospy.Subscriber('/WPD/Speed', TwistStamped, command_calback)
rospy.spin()
