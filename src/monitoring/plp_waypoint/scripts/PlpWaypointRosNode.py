import rospy
import plp_waypoint.msg
from robil_msgs.msg import Map, Path
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped


PLP_TOPIC = "/plp/messages"

TODO_VALUE = "todo value"

class PlpWaypointRoseNode:
    """
    A harness for a PlpWaypoint in a ROS/RobIL system. Listens to the right topics, feeds the data to
    the PLP objects, and emits predictions when possible.
    """

    def __init__(self):
        # Init internal PLP
		self.plp = PlpWaypoint( {"MIN_LOC_ERROR": 5, #m
								 "BOBCAT_SIZE": (3.5, 2, 1.7),  # L x H x W, in meters
								 "MIN_BLADE_CLEARANCE": 1,  # m
								 "FUEL_CONSUMPTION_RATE": 10000, # m/liter
                                 "BOBCAT_AVERAGE_SPEED": 20*1000 # m/hour
                                 })
        self.trigger_state = 0
        
        #Init the ROS stuff
        rospy.init_node("PlpWaypoint", anonymous=False)
        self.publisher = rospy.Publisher(PLP_TOPIC, PlpMessage, queue_size=5)
        rospy.Subscriber("/PER/MiniMap", Map, self.map_updated)
        rospy.Subscriber("/PP/Path", Path, self.path_updated)
        rospy.Subscriber("/Loc/Pose", PoseWithCovarianceStamped, self.position_updated)
        rospy.Subscriber(TODO_VALUE, TODO_VALUE, self.nav_task_active)
        rospy.Subscriber(TODO_VALUE, TODO_VALUE, self.receive_local_path)
    
    def path_updated(self, msg):
        self.plp.update_path(msg)
    
    def map_updated(self, msg):
        self.plp.update_map(msg)
    
    def position_updated(self, msg):
        self.plp.update_position(msg)
    
    def nav_task_active(self, msg):
        self.advace_trigger_state()
    
    def receve_local_path(self, msg):
        self.advace_trigger_state()
    
    def advance_trigger_state(self):
        self.state = self.state + 1
        if ( self.state == 2 ):
            self.attempt_estimation()
            self.state = 0
        
    def attempt_estimation(self):
        """
        Attempts to get estimation from the PLP. May fail if the PLP does not have
        enough information.
        """
        if self.plp.can_estimate:
            res = self.plp.get_estimation()
            self.publisher.pubish( PlpMessage(None, "Waypoint", "Estimation", repr(res)))
        else
            self.publisher.publish( PlpMessage(None, "Waypoint", "error", "PLP triggered, but not enough data available"))
        

        
    