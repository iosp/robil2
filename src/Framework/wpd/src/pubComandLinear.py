import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32

WPD_command_pub = rospy.Publisher('/WPD_command', Float32 , queue_size=5)

def command_calback(msg):
   msgToSnd = Float32()
   msgToSnd.data = msg.twist.linear.x
   WPD_command_pub.publish(msgToSnd)

rospy.init_node('wpd_command_pub')
rospy.Subscriber('/WPD/Speed', TwistStamped, command_calback)
rospy.spin()
