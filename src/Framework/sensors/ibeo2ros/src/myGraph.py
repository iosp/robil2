from math import pi
import numpy
import matplotlib
import pylab as plt
import rospy
from robil_msgs.msg import MultiLaserScan

import signal, sys

def signal_handler(signal, frame):
    plt.ioff()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

cnt = 0
def plot_points(points):
    global cnt
    if cnt == 0:
      angb1 = []
      angb2 = []
      for i in range(len(points.ranges_b1)):
	angb1.append(points.angle_min_b+i*points.angle_increment)
      for i in range(len(points.ranges_b2)):
	angb2.append(points.angle_min_b+i*points.angle_increment)
      angt1 = []
      for i in range(len(points.ranges_t1)):
	angt1.append(points.angle_min_t+i*points.angle_increment)
      angt2 = []
      for i in range(len(points.ranges_t2)):
	angt2.append(points.angle_min_t+i*points.angle_increment)
      fig1 = plt.figure(1)
      plt.ion()
      plt.clf()
      ax = plt.subplot(111,polar=True)
      plt.scatter(angb1,points.ranges_b1,color='b')
      plt.scatter(angb2,points.ranges_b2,color='g')
      plt.scatter(angt1,points.ranges_t1,color='r')
      plt.scatter(angt2,points.ranges_t2,color='y')
      plt.draw()
      plt.ioff()
    else:
      if cnt == 5:
	cnt = -1
    cnt += 1

print "starting GUI"
rospy.init_node('ibeo_point_reader')
sub = rospy.Subscriber('/SENSORS/IBEO/2',MultiLaserScan,plot_points)
print "Subscribing to ",(sub.name)
rospy.spin()