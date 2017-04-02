#!/usr/bin/env python
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
publ = rospy.Publisher('/LLC/EFFORTS/Throttle', Float64, queue_size=10)
pubr = rospy.Publisher('/LLC/EFFORTS/Steering', Float64, queue_size=10)
v_l=0
v_r=0
rospy.init_node('cmd_vel_listener')
rate = rospy.Rate(20)
def callback(msg):

    v_l = msg.linear.x
    v_r = msg.angular.z
    # rospy.loginfo(v_l)
    publ.publish(Float64(v_l))
    pubr.publish(Float64(v_r))
    

def listener(): 
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    
    try:
        listener()
    except rospy.ROSInterruptException:
        pass