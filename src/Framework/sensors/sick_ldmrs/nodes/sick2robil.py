#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from robil_msgs.msg import MultiLaserScan
import message_filters

pub = rospy.Publisher('/SENSORS/IBEO/2', MultiLaserScan, queue_size = 1)


def callback(sick0, sick1, sick2, sick3):
    msg = MultiLaserScan()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'ibeo'
    msg.angle_min_b = sick0.angle_min
    msg.angle_max_b = sick0.angle_max
    msg.angle_min_t = sick3.angle_min
    msg.angle_max_t = sick3.angle_max
    msg.angle_increment = sick0.angle_increment
    msg.scan_time = sick0.scan_time
    msg.angle_t1 = 0.014
    msg.angle_t2 = 0.028
    msg.angle_b1 = -0.014
    msg.angle_b2 = -0.028    
    msg.ranges_b2 = sick0.ranges
    msg.ranges_b1 = sick1.ranges
    msg.ranges_t1 = sick2.ranges
    msg.ranges_t2 = sick3.ranges    
    msg.range_min = 0.1
    msg.range_max = 80        
    
    pub.publish(msg)
    

if __name__ == '__main__':
    rospy.init_node('sick2robil')
    s0 = message_filters.Subscriber('/sickldmrs/scan0', LaserScan)
    s1 = message_filters.Subscriber('/sickldmrs/scan1', LaserScan)
    s2 = message_filters.Subscriber('/sickldmrs/scan2', LaserScan)
    s3 = message_filters.Subscriber('/sickldmrs/scan3', LaserScan)            
    
    ts = message_filters.TimeSynchronizer([s0, s1, s2, s3], 10)
    ts.registerCallback(callback)
    print('Publishing ROBIL Multilaser scan message')
    rospy.spin()
