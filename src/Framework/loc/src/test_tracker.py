import rospy
from geometry_msgs.msg._PoseWithCovarianceStamped import PoseWithCovarianceStamped
import simpleguitk as simplegui
from geometry_msgs.msg import PoseWithCovarianceStamped,TwistStamped
from sensor_msgs.msg import NavSatFix
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
import numpy

WIDTH = 500
LENGTH = 500
name = 'drivingVic'
init_X_Y = [0,0]
E_loc = []
class location:
    def __init__(self):
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._speed_x = 0.0
        self._speed_y = 0.0
        self._acc_x = 0.0
        self._acc_y = 0.0
        self._gy = 0.0

class ekfDataLog:
    def __init__(self):
        rospy.init_node('tracker')
        self._real_loc = location()
        self._estimator = location()
        self._loc_sub = rospy.Subscriber('/LOC/Pose',PoseWithCovarianceStamped, self.estimatedLocation)
        self._speed_sub = rospy.Subscriber('/LOC/Velocity',TwistStamped,self.estimatedSpeed)
        self._real_loc_sub = rospy.Subscriber('/gazebo/model_states',ModelStates, self.realLocation)
        self.init_xy = [0,0]

    def realLocation(self,obj):
        i = 0
        for objecti in obj.name:
            if objecti == name:
                val = i
                break
            i += 1
        theta = euler_from_quaternion( numpy.array((obj.pose[val].orientation.x, obj.pose[val].orientation.y, obj.pose[val].orientation.z, obj.pose[val].orientation.w), dtype=numpy.float64))
        self._real_loc._x = obj.pose[val].position.x - init_X_Y[0]
        self._real_loc._y = obj.pose[val].position.y - init_X_Y[1]
        self._real_loc._yaw = theta[2]
        self._real_loc._speed_x = obj.twist[val].linear.x
        self._real_loc._speed_y = obj.twist[val].linear.y
        self._real_loc._gy = obj.twist[val].angular.z
        self.init_xy = [obj.pose[val].position.x,obj.pose[val].position.y]

    def estimatedLocation(self,obj):
        self._estimator._x = obj.pose.pose.position.x
        self._estimator._y = obj.pose.pose.position.y
        theta = euler_from_quaternion( numpy.array((obj.pose.pose.orientation.x, obj.pose.pose.orientation.y, obj.pose.pose.orientation.z, obj.pose.pose.orientation.w), dtype=numpy.float64))
        self._estimator._yaw = theta[2]

    def estimatedSpeed(self,obj):
        self._estimator._speed_x = obj.twist.linear.x
        self._estimator._speed_y = obj.twist.linear.y

    def error_calculator(self):
        e_x = self._estimator._x - self._real_loc._x
        e_y = self._estimator._y - self._real_loc._y                
        e_yaw = self._estimator._yaw - self._real_loc._yaw
        e_s_x = self._estimator._speed_x - self._real_loc._speed_x
        e_s_y = self._estimator._speed_y - self._real_loc._speed_y
        return e_x,e_y,e_yaw,e_s_x,e_s_y

    	

ros_log = ekfDataLog()

def draw(canvas):
    canvas.draw_text('Estimated location', (10,20),12,'Red')
    canvas.draw_text('------------------', (10,35),12,'Red')
    stri = 'x     : %.2f\ny     : %.2f\nyaw: %.2f\nspeed_x: %.2f\nspeed_y: %.2f'%((ros_log._estimator._x),(ros_log._estimator._y),(ros_log._estimator._yaw),ros_log._estimator._speed_x,ros_log._estimator._speed_y)
    canvas.draw_text(stri, (10,130),12,'Red')

    canvas.draw_text('Real location', (260,20),12,'Blue')
    canvas.draw_text('-------------', (260,35),12,'Blue')
    stri = 'x     : %.2f\ny     : %.2f\nyaw: %.2f\nspeed_x: %.2f\nspeed_y: %.2f'%((ros_log._real_loc._x),(ros_log._real_loc._y),(ros_log._real_loc._yaw),ros_log._real_loc._speed_x,ros_log._real_loc._speed_y)
    canvas.draw_text(stri, (260,130),12,'Blue')

    canvas.draw_text('Error', (10,270),12,'Green')
    canvas.draw_text('--------', (10,285),12,'Green')
    stri = 'x     : %.2f\ny     : %.2f\nyaw: %.2f\nspeed_x: %.2f\nspeed_y: %.2f'%(ros_log.error_calculator())
    canvas.draw_text(stri, (10,370),12,'Green')    

def change_init():
    global init_X_Y
    init_X_Y = ros_log.init_xy

frame = simplegui.create_frame("EKF performance", WIDTH, LENGTH)
frame.set_canvas_background('White')
label = frame.add_label('Data tracker')
frame.set_draw_handler(draw)
frame.add_button('Change init',change_init)
#frame.add_button('E(GPS)',E_GPS)
#loc_timer = simplegui.create_timer(100, location_handler)
#loc_timer.start()
frame.start()
