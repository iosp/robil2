import rospy
import math
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32

velocity_linear_command_pub = rospy.Publisher('/paltform_velocity_monitor/linear', Float32 , queue_size=5)
velocity_angular_command_pub = rospy.Publisher('/paltform_velocity_monitor/angular', Float32 , queue_size=5)

def command_calback(msg):
	names = msg.name
	if 'Sahar' in names:
		index = names.index('Sahar')
	else:
		print 'Sahar dosn\'t exist in the Gazebo'
	
	velocityToPub = Float32()
	squareOfX = math.sqrt(math.fabs(msg.twist[index].linear.x))
	squareOfY = math.sqrt(math.fabs(msg.twist[index].linear.y))
	velocityToPub.data = math.pow(squareOfX + squareOfY, 0.5)
	velocity_linear_command_pub.publish(velocityToPub)

	velocity_angular_command_pub.publish(msg.twist[index].angular.z)

rospy.init_node('velocity_command_pub')
rospy.Subscriber('/gazebo/model_states', ModelStates, command_calback)
rospy.spin()
