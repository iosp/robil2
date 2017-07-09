#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, TwistStamped

def callback(msg=Twist):
    new_msg = TwistStamped()
    new_msg.twist = msg
    pub.publish(new_msg)

rospy.init_node('drive_bob')
rospy.Subscriber('/bob', Twist, callback)
pub = rospy.Publisher('/WPD/Speed', TwistStamped, queue_size=1)
rospy.spin()
