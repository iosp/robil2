from math import pi
import numpy

class Point:
    
    def __init__(self, layerEcho, flags, horizAngle, RadialDist,echoPulseWidth):
	self._flags = flags
	self._layerEcho = layerEcho
	self._horizAngle = horizAngle
	self._RadialDist = RadialDist
	self._EchoPulseWidth = echoPulseWidth
	
	self._layer = 0
	self._echo = 0
	self.processData()
	
    def processData(self):
	self._layer = self._layerEcho%4
	self._echo = (self._layerEcho - self._layer)/16


from robil_msgs.msg import MultiLaserScan
import rospy
class MeasureData:
  
    def __init__(self):
      self._rosPointArray = MultiLaserScan()
      self._angles1 = []
      self._angles2 = []
      self._angles3 = []
      self._angles4 = []
    
    def reset_angles(self):
      self._rosPointArray = MultiLaserScan()
      self._angles1 = []
      self._angles2 = []
      self._angles3 = []
      self._angles4 = []
    def set_point_array_as_ROS(self,pointArray,startAngle,endAngle,step):
      self.reset_angles()
      for point in pointArray:
	if point._layer == 0:
	  self._rosPointArray.ranges_b1.append(point._RadialDist/100.0)
	  self._angles1.append(2*pi*point._horizAngle/step)
	elif point._layer == 1:
	  self._rosPointArray.ranges_b2.append(point._RadialDist/100.0)
	  self._angles2.append(2*pi*point._horizAngle/step)
	elif point._layer == 2:
	  self._rosPointArray.ranges_t2.append(point._RadialDist/100.0)
	  self._angles3.append(2*pi*point._horizAngle/step)
	elif point._layer == 3:
	  self._rosPointArray.ranges_t1.append(point._RadialDist/100.0)
	  self._angles4.append(2*pi*point._horizAngle/step)
	  
      self._rosPointArray.header.frame_id = 'ibeo'
      self._rosPointArray.header.stamp = rospy.Time.now()

      self._rosPointArray.angle_t1 = 0.014
      self._rosPointArray.angle_t2 = 0.028
      self._rosPointArray.angle_b1 = -0.014
      self._rosPointArray.angle_b2 = -0.028      
      
      self._rosPointArray.range_min = min(min(self._rosPointArray.ranges_b1),min(self._rosPointArray.ranges_b2)
					  ,min(self._rosPointArray.ranges_t1),min(self._rosPointArray.ranges_t2))
      self._rosPointArray.range_max = max(max(self._rosPointArray.ranges_b1),max(self._rosPointArray.ranges_b2)
					  ,max(self._rosPointArray.ranges_t1),max(self._rosPointArray.ranges_t2))
      
      self._rosPointArray.angle_increment = (self._angles1[31] - self._angles1[30])
      self._rosPointArray.angle_min_t = min(min(self._angles3),min(self._angles4))
      self._rosPointArray.angle_max_t = max(max(self._angles3),max(self._angles4))
      self._rosPointArray.angle_min_b = min(min(self._angles1),min(self._angles2))
      self._rosPointArray.angle_max_b = max(max(self._angles1),max(self._angles2))