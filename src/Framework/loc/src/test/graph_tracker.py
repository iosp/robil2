import thread
import rospy
from geometry_msgs.msg._PoseWithCovarianceStamped import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped,TwistStamped
from sensor_msgs.msg import NavSatFix
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
import numpy
import matplotlib
import pylab as plt
import signal
import sys

WIDTH = 500
LENGTH = 500
name = 'Sahar'
init_X_Y = [0,0,0]
real_x_log = []
x_log = []
real_y_log = []
y_log = []
speed_x_log = []
speed_y_log = []
real_speed_x_log = []
real_speed_y_log = []
plot_on = 1

class location:
    def __init__(self):
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._yaw = 0.0
        self._speed_x = 0.0
        self._speed_y = 0.0
        self._acc_x = 0.0
        self._acc_y = 0.0
        self._gy = 0.0

class ekfDataLog:
    def __init__(self):
        rospy.init_node('graph_tracker')
        self._real_loc = location()
        self._estimator = location()
        self._loc_sub = rospy.Subscriber('/LOC/Pose',PoseWithCovarianceStamped, self.estimatedLocation)
        self._speed_sub = rospy.Subscriber('/LOC/Velocity',TwistStamped,self.estimatedSpeed)
        self._real_loc_sub = rospy.Subscriber('/gazebo/model_states',ModelStates, self.realLocation)
        self.init_xyz = [0,0]

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
        self._real_loc._z = obj.pose[val].position.z
        self._real_loc._yaw = theta[2]
        self._real_loc._speed_x = obj.twist[val].linear.x
        self._real_loc._speed_y = obj.twist[val].linear.y
        self._real_loc._gy = obj.twist[val].angular.z
        self.init_xyz = [obj.pose[val].position.x,obj.pose[val].position.y]

    def estimatedLocation(self,obj):
        self._estimator._x = obj.pose.pose.position.x
        self._estimator._y = obj.pose.pose.position.y
        self._estimator._z = obj.pose.pose.position.z
        theta = euler_from_quaternion( numpy.array((obj.pose.pose.orientation.x, obj.pose.pose.orientation.y, obj.pose.pose.orientation.z, obj.pose.pose.orientation.w), dtype=numpy.float64))
        self._estimator._yaw = theta[2]

    def estimatedSpeed(self,obj):
        self._estimator._speed_x = obj.twist.linear.x
        self._estimator._speed_y = obj.twist.linear.y

    def error_calculator(self):
        e_x = self._estimator._x - self._real_loc._x
        e_y = self._estimator._y - self._real_loc._y                
        e_z = self._estimator._z - self._real_loc._z
        e_yaw = self._estimator._yaw - self._real_loc._yaw
        e_s_x = self._estimator._speed_x - self._real_loc._speed_x
        e_s_y = self._estimator._speed_y - self._real_loc._speed_y
        return e_x,e_y,e_z,e_yaw,e_s_x,e_s_y

num = 50
num1 = 50
num2 = 50
biasx = 0
biasy = 0
on1 = False
on2 = False
ros_log = ekfDataLog()
end = False

def print_menu():
    print "Localization Graphs"
    print "-------------------"
    print "1. Location Graph (turn on/off)"
    print "2. Speed Graph (turn on/off)"
    print "3. Default (1 & 2 with 10 readings)"
    print "4. Add bias to real location"
    print "5. Reset bias"
    print "To Exit press - ctrl+C"
    return int ( raw_input('Enter your choice [1-3] : ') )
def handle_inp(delay):
    global num1,num2,on1,on2,biasx,biasy
    try:
        while(not end):
            choice = print_menu()
            if choice == 1:
                if not on1:
                    num1 = int( raw_input('How many Readings per graph? ') )
                    on1 = True

                else:
                    on1 = False
            elif choice == 2:
                if not on2:
                    num2 = int( raw_input('How many Readings per graph? ') )
                    on2 = True
                else:
                    on2 = False
            elif choice == 3:
                on1 = True
                on2 = True
                num1 = 10
                num2 = 10
            elif choice == 4: 
            	biasx += float( raw_input('Add bias x? ') )
            	biasy += float( raw_input('Add bias y? ') )
            elif choice == 5:
            	biasx = 0
            	biasy = 0
            else:
                print "Wrong selection"


    except:
        print "thread crashed. Press ctrl+C"

def signal_handler(signal, frame):
    global end
    end = True
    print 'Bye'
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
thread.start_new_thread( handle_inp, (0.01, ) )
once1 = False
once2 = False
while(1):
        real_x_log.append(ros_log._real_loc._x + biasx)
        x_log.append(ros_log._estimator._x)
        real_y_log.append(ros_log._real_loc._y + biasy)
        y_log.append(ros_log._estimator._y)
        speed_x_log.append(ros_log._estimator._speed_x)
        speed_y_log.append(ros_log._estimator._speed_y)
        real_speed_x_log.append(ros_log._real_loc._speed_x)
        real_speed_y_log.append(ros_log._real_loc._speed_y)
        while len(real_x_log) > num1:
            real_x_log.pop(0)
        while len(real_y_log) > num1:
            real_y_log.pop(0)
        while len(x_log) > num1:
            x_log.pop(0)
        while len(y_log) > num1:
            y_log.pop(0)
        while len(real_speed_x_log) > num2:
            real_speed_x_log.pop(0)
        while len(speed_x_log) > num2:
            speed_x_log.pop(0)
        while len(real_speed_y_log) > num2:
            real_speed_y_log.pop(0)
        while len(speed_y_log) > num2:
            speed_y_log.pop(0)
        fig1 = plt.figure(1)
        if on1:
            once1 = True
            plt.ion()
            plt.clf()
            plt.subplot(2,1,1)
            plt.plot(real_x_log)
            plt.plot(x_log)
            plt.title("x")
            plt.subplot(2,1,2)
            plt.plot(real_y_log)
            plt.plot(y_log)
            plt.title("y")
            plt.draw()
            plt.ioff()

        else:
            if once1:
                plt.close()
                once1 = False
        plt.figure(2)
        if on2:
            once2 = True
            plt.ion()
            plt.clf()
            plt.subplot(2,1,1)
            plt.plot(real_speed_x_log)
            plt.plot(speed_x_log)
            plt.title("v_x")
            plt.subplot(2,1,2)
            plt.plot(real_speed_y_log)
            plt.plot(speed_y_log)
            plt.title("v_y")
            plt.draw()
            plt.ioff()
        else:
            if once2:
                plt.close()
                once2 = False

        

