import rospy
from sensor_msgs.msg import PointCloud
from robil_msgs.msg import MultiLaserScan
from geometry_msgs.msg import Point32
import tf
from math import sin, cos

pcpub = rospy.Publisher('/SENSORS/IBEO/PC', PointCloud, queue_size=5)

def cb(msg=MultiLaserScan()):
    pc = PointCloud()
    for ray in range(0,4):
        if ray == 0:
            arr = msg.ranges_t1
            phi = msg.angle_t1
        elif ray == 1:
            arr = msg.ranges_t2
            phi = msg.angle_t2
        elif ray == 2:
            arr = msg.ranges_b1
            phi = msg.angle_b1
        else:
            arr = msg.ranges_b2
            phi = msg.angle_b2
        for i in range(len(arr)):
            p = Point32()
            p.x = arr[i] * cos(msg.angle_max_t - i * msg.angle_increment) * cos(phi)
            p.y = arr[i] * sin(msg.angle_max_t - i * msg.angle_increment) * cos(phi)
            p.z = arr[i] * sin(phi)
            pc.points.append(p)
    pc.header.stamp = rospy.Time.now()
    pc.header.frame_id = 'ibeo'
    pcpub.publish(pc)



rospy.init_node('ib2pc')
rospy.Subscriber('/SENSORS/IBEO/1', MultiLaserScan, cb)
rospy.spin()