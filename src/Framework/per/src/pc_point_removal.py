#!/usr/bin/env python
import rospy
from dynamic_reconfigure.msg import Config
from sensor_msgs.msg import PointCloud

height = 0
pub = rospy.Publisher('/PC', PointCloud, queue_size=1)

def callback(config):
    global height
    print 'change', config.doubles[0]
    height = config.doubles[0].value
    #global height_thresh=height


def callback2(pc=PointCloud()):
    avg_height = 0
    pc_new = PointCloud()
    for p in pc.points:
        avg_height += (p.z + 2.314)
        if (p.z+ 2.314) > height:
            pc_new.points.append(p)
    pc_new.header.stamp = rospy.Time.now()
    pc_new.header.frame_id = pc.header.frame_id
    pub.publish(pc_new)
    print 'Recived ', len(pc.points), ' points. avg height is: ', avg_height / len(pc.points), '     published ', len(pc_new.points), ' points'



if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous = True)

    rospy.Subscriber('/per_node/parameter_updates', Config, callback)

    #rospy.init_node('listener', annonymouse = True)
    rospy.Subscriber('/SENSORS/IBEO/WORLDPC', PointCloud, callback2)
    rospy.spin() #keeps python from exiting until this node is stopped.