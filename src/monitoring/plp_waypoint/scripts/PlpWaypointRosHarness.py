#!/usr/bin/env python
import rospy
from robil_msgs.msg import Map, Path, AssignNavTask, AssignMission
# from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance

# from plp_waypoint.msg import PlpMessage
from std_msgs.msg import String, Header # TODO replace with the plp message

from PlpWaypoint import *

PLP_TOPIC = "/plp/messages"

class PlpWaypointRosHarness:
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
                                 "BOBCAT_AVERAGE_SPEED": 20000 # m/hour
                                 } )

        # Init harness state
        self.trigger_state = 0
        self.nav_tasks = {} # id -> nav_task
        self.missions = {} # id -> mission

        #Init the ROS stuff
        rospy.init_node("plp_waypoint", anonymous=False)
        self.publisher = rospy.Publisher(PLP_TOPIC, String, queue_size=5)
        rospy.Subscriber("/PER/MiniMap", Map, self.map_updated)
        rospy.Subscriber("/PP/Path", Path, self.path_updated)
        rospy.Subscriber("/Loc/Pose", PoseWithCovarianceStamped, self.position_updated)
        rospy.Subscriber("/OCU/SMME/NavigationTask", AssignNavTask, self.nav_task_assigned)
        rospy.Subscriber("/OCU/SMME/MissionPlan", AssignMission, self.mission_assigned)
        rospy.Subscriber("/decision_making/events", String, self.state_machine_change)

        rospy.loginfo("Started")

    def path_updated(self, msg):
        # rospy.loginfo("PlpWaypointRosHarness: Updating Path")
        self.plp.update_path(msg)

    def map_updated(self, msg):
        # rospy.loginfo("PlpWaypointRosHarness: Updating Map")
        self.plp.update_map(msg)

    def position_updated(self, msg):
        rospy.loginfo("Position updated")
        self.plp.update_position(msg)

    def nav_task_active(self, msg):
        rospy.loginfo("Task is active!")
        self.advace_trigger_state()

    def receive_local_path(self, msg):
        rospy.loginfo("Local path received")
        self.advace_trigger_state()

    def nav_task_assigned(self, nav_task):
        rospy.loginfo("navtask {0} stored".format(nav_task.task_id))
        self.nav_tasks[ nav_task.task_id ] = nav_task

    def mission_assigned(self, mission):
        rospy.loginfo("mission {0} stored".format(mission.mission_id))
        self.missions[ mission.mission_id ] = mission

    def state_machine_change(self, eventString):
        rospy.loginfo("evt: {0}".format(eventString))

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
        rospy.loginfo("Activating PLP")
        if self.plp.can_estimate:
            res = self.plp.get_estimation()
            self.publisher.pubish( std_msgs.msg.String(repr(PlpMessage(None, "Waypoint", "Estimation", repr(res))))) # TODO remove repr when proper message is used
        else:
            self.publisher.publish( std_msgs.msg.String(repr(PlpMessage(None, "Waypoint", "error", "PLP triggered, but not enough data available")))) # TODO remove repr when proper message is used


if __name__ == '__main__':
    try:
        rospy.loginfo("Starting plp_waypoint node...")
        harness = PlpWaypointRosHarness()

        # DuckTape: Adding a fake position. TODO: remove once /Loc/Pose is active
        harness.position_updated( PoseWithCovarianceStamped() )

        rospy.loginfo("started")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
