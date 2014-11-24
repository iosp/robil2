import rospy
from robil_msgs.msg import Map

def subscriber_call(data):
  row = 20
  col = 15
  print (data.data[row*30+col])

rospy.init_node('listener', anonymous=True)

rospy.Subscriber("/PER/MiniMap", Map, subscriber_call)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()