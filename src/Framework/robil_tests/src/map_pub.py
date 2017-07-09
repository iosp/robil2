import rospy
import math
import tf
from robil_msgs.msg import Map
from robil_msgs.msg import MapCell
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

map_pub = rospy.Publisher('/PER/Map', Map , queue_size=5)


obs_list = [(0,0) , (0,50) ,  (0,100)] 
trans = Point(0,0,0)
rot = Quaternion(0,0,0,1)

def pub_msg():
        global trans
        global rot
        
	map = Map()
	map.header.stamp = rospy.Time.now()
	map.header.frame_id = "/body"
	
	map.info.map_load_time = rospy.Time.now()
	map.info.resolution = 0.20
	map.info.width = 150
	map.info.height = 150
	
	print "good TF !!"
	
	map.info.origin.position = Point(trans[0],trans[1],trans[2])
	map.info.origin.orientation = Quaternion(rot[0],rot[1],rot[2],rot[3])

	#print map.info.origin.orientation
	
        W = map.info.width
        H = map.info.height
        cell_list = []
	for i in range(0, H) :
	    for j in range(0, W) :
		cell = MapCell()
		cell.height = 0
		cell.type = 1
		cell.feature = 0
		
		if (i,j) in obs_list :
		    cell.type = 2
		    
		
		map.data.append(cell)
			
	map_pub.publish(map)


def main():
	rate = rospy.Rate(10)
	listener = tf.TransformListener()
        global trans
        global rot
        
	while not rospy.is_shutdown():
	  try:
	     (trans,rot) = listener.lookupTransform('/body', '/world_gazebo' ,  rospy.Time(0))
	  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	      print "failed to fined TF"
	      continue
	   
	  pub_msg()
	  rate.sleep()
		
	
rospy.init_node('map_pub_node')

main()




