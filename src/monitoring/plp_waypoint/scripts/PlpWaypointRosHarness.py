#!/usr/bin/env python
import rospy
from robil_msgs.msg import Map, Path, AssignNavTask, AssignMission
# from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped

from plp_waypoint.msg import PlpMessage
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
        # The constants table is defined in the PLP document under "Values/Constants"
        self.plp = PlpWaypoint( {"MIN_LOC_ERROR": 5, # meters
                                 "BOBCAT_SIZE": (3.5, 2, 1.7),  # L x H x W, in meters
                                 "MIN_BLADE_CLEARANCE": 1,  # meters
                                 "FUEL_CONSUMPTION_RATE": 10000, # m/liter
                                 "BOBCAT_AVERAGE_SPEED": 20000, # m/hour
                                 "RATE_PATH_LENGTH": 0.85, # 0..1
                                 "RATE_AERIAL_DISTANCE": 0.95 # 0..1
                                 } )

        # Init harness state
        self.trigger_nav_task_active = False
        self.trigger_local_path_published = False
        self.nav_tasks = {} # id -> nav_task
        self.missions = {} # id -> mission
        self.mission_state = {} # mission_id -> current task index.
        self.monitor_active = False

        #Init the ROS stuff
        rospy.init_node("plp_waypoint", anonymous=False)
        self.publisher = rospy.Publisher(PLP_TOPIC, PlpMessage, queue_size=5)
        rospy.Subscriber("/PER/MiniMap", Map, self.map_updated)
        rospy.Subscriber("/PP/Path",
                    Path, self.path_updated)
        rospy.Subscriber("/LOC/Pose",
                    PoseWithCovarianceStamped, self.position_updated)
        rospy.Subscriber("/OCU/SMME/NavigationTask",
                    AssignNavTask, self.nav_task_assigned)
        rospy.Subscriber("/OCU/SMME/MissionPlan",
                    AssignMission, self.mission_assigned)
        rospy.Subscriber("/decision_making/events",
                    String, self.state_machine_change)

        rospy.loginfo("Started")

    def path_updated(self, msg):
        # rospy.loginfo("PlpWaypointRosHarness: Updating Path")
        self.plp.update_path(msg)
        self.trigger_local_path_published = True
        self.attempt_estimation()
        if self.monitor_active:
            rospy.loginfo("remaning path:{0}pts {1}m"
                    .format(len(self.plp.path.waypoints.poses),
                            self.plp.calc_local_path_distance()))

    def map_updated(self, msg):
        # rospy.loginfo("PlpWaypointRosHarness: Updating Map")
        self.plp.update_map(msg)

    def position_updated(self, msg):
        # rospy.loginfo("Position updated")
        self.plp.update_position(msg)

    def nav_task_assigned(self, nav_task):
        rospy.loginfo("navtask {0} stored".format(nav_task.task_id))
        self.nav_tasks[ nav_task.task_id ] = nav_task

    def mission_assigned(self, mission):
        rospy.loginfo("mission {0} stored".format(mission.mission_id))
        self.missions[ mission.mission_id ] = mission

    def state_machine_change(self, eventString):
        comps = eventString.data.split("/")
        # Test for the triggering of a task.
        if (len(comps) == 8 and
            comps[1]=="mission" and
            comps[3]=="TaskActive" and
            comps[5]=="TaskSpooling"):
            mission_id = comps[2]

            if ( self.mission_state.has_key( mission_id ) ):
                self.mission_state[mission_id] += 1
            else:
                self.mission_state[mission_id] = 0

            task_index = self.mission_state[mission_id]

            # test if the task is a navigtion task
            rospy.loginfo("Task #{0} of mission {1} spooling".format(task_index, mission_id) )
            if self.missions.has_key(mission_id):
                mission = self.missions[mission_id]
                task_id = mission.tasks[task_index].task_id
                if self.nav_tasks.has_key(task_id):
                    rospy.loginfo("Will attempt calculation once local path is ready")
                    self.trigger_local_path_published = False
                    self.trigger_nav_task_active = True

        elif ( len(comps) == 4 and
               comps[1]=="mission") :
            mission_id = comps[2]
            event = comps[3]
            if ( event in { "AbortMission", "CompleteMission", "ClearMission"}):
                if (self.mission_state.has_key(mission_id) ):
                    del self.mission_state[mission_id]


    def attempt_estimation(self):
        """
        Attempts to get estimation from the PLP. May fail if the PLP does not have
        enough information.
        """
        if ( self.trigger_nav_task_active and self.trigger_local_path_published ):
            # turm trigger off
            self.trigger_local_path_published = False
            self.trigger_nav_task_active = False

            # Estimate and publish.
            rospy.loginfo("Activating PLP")
            self.monitor_active = True  # TODO turm the monitor off when no nav task is active
            if self.plp.can_estimate:
                res = self.plp.get_estimation()
                self.publisher.publish( PlpMessage(None, "Waypoint", "Estimation", repr(res)) )
            else:
                self.publisher.publish( PlpMessage(None, "Waypoint", "error", "PLP triggered, but not enough data available") )


if __name__ == '__main__':
    try:
        rospy.loginfo("Starting plp_waypoint node...")
        harness = PlpWaypointRosHarness()

        rospy.loginfo("started")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
