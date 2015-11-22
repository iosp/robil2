#!/usr/bin/env python
import rospy
from robil_msgs.msg import Map, Path, AssignNavTask, AssignMission
# from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped

from plp_waypoint.msg import PlpMessage
from std_msgs.msg import String, Header # TODO replace with the plp message

from PlpWaypoint import *
from PlpWaypointClasses import *

PLP_TOPIC = "/plp/messages"

class PlpWaypointRosHarness(object):
    """ A harness for a PlpWaypoint in a ROS/RobIL system. Listens to the
        right topics, feeds the data to the PLP objects, and emits predictions
        when possible."""

    def __init__(self):
        # The constants table is defined in the PLP document
        # under "Values/Constants"
        self.plp_constants = {
                                "MIN_LOC_ERROR": 5, # meters
                                "BOBCAT_SIZE": (3.5, 2, 1.7),  # LxHxW, meters
                                "MIN_BLADE_CLEARANCE": 1,  # meters
                                "FUEL_CONSUMPTION_RATE": 10000, # m/liter
                                "BOBCAT_AVERAGE_SPEED": 20000, # m/hour
                                "RATE_PATH_LENGTH": 0.85, # 0..1
                                "RATE_AERIAL_DISTANCE": 0.95 # 0..1
                            }

        self.nav_tasks = {} # id -> nav_task
        self.missions = {} # id -> mission
        self.mission_state = {} # mission_id -> current task index.

        self.trigger_nav_task_active = False
        self.trigger_local_path_published = False
        self.plp = None
        self.plp_params = PlpWaypointParameters()

        # Init the ROS stuff
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

        rospy.loginfo("Waypoinrt PLP Harness - Started")


    def path_updated(self, msg):
        self.plp_params.set_path(msg)
        self.trigger_local_path_published = True
        self.consider_trigger()

    def map_updated(self, msg):
        self.plp_params.set_map(msg)

    def position_updated(self, msg):
        self.plp_params.set_position(msg)

    def nav_task_assigned(self, nav_task):
        rospy.loginfo("navtask {0} stored".format(nav_task.task_id))
        self.nav_tasks[nav_task.task_id] = nav_task

    def mission_assigned(self, mission):
        rospy.loginfo("mission {0} stored".format(mission.mission_id))
        self.missions[mission.mission_id] = mission

    def state_machine_change(self, event_string):
        comps = event_string.data.split("/")
        # Test for the triggering of a task.
        if (len(comps) == 8 and
            comps[1] == "mission" and
            comps[3] == "TaskActive" and
            comps[5] == "TaskSpooling" ) :
            mission_id = comps[2]

            if self.mission_state.has_key(mission_id):
                self.mission_state[mission_id] += 1
            else:
                self.mission_state[mission_id] = 0

            task_index = self.mission_state[mission_id]

            # test if the task is a navigtion task
            rospy.loginfo(
                "Task #{0} of mission {1} spooling".format(task_index, mission_id))
            if self.missions.has_key(mission_id):
                mission = self.missions[mission_id]
                task_id = mission.tasks[task_index].task_id
                if self.nav_tasks.has_key(task_id):
                    rospy.loginfo("Will attempt calculation once local path is ready")
                    self.trigger_local_path_published = False
                    self.trigger_nav_task_active = True

        elif (len(comps) == 4 and
               comps[1] == "mission") :
            mission_id = comps[2]
            event = comps[3]
            if event in {"AbortMission", "CompleteMission", "ClearMission"}:
                if self.mission_state.has_key(mission_id):
                    del self.mission_state[mission_id]
                    self.reset_harness_data()


    def consider_trigger(self):
        """Test the status of the fields. If all preconditions are met,
            trigger the plp."""
        if self.trigger_nav_task_active and self.trigger_local_path_published:
            self.trigger_local_path_published = False
            self.trigger_nav_task_active = False
            self.trigger_plp_task()


    def trigger_plp_task(self):
        """Creates a PLP and starts the monitoring, if there's no PLP yet."""
        rospy.loginfo("Activating PLP")
        self.plp = PlpWaypoint(self.plp_constants, self.plp_params, self)
        self.plp.request_estimation()

    def reset_harness_data(self):
        self.plp = None
        self.plp_params.callback = None
        self.plp_params = PlpWaypointParameters()
        self.trigger_local_path_published = False
        self.trigger_nav_task_active = False


    # PLP Callback methods #####################################
    def plp_estimation(self, plp_achieve_result):
        """ The PLP is active, and gives an estimation. """
        self.publisher.publish(
            PlpMessage(None, "Waypoint", "estimation",
                        repr(plp_achieve_result)))

    def plp_goal_achieved(self):
        """ The goal for the PLP has been achieved.
            Deletes the current PLP, resets the harness. """
        rospy.loginfo("PLP goal achieved")
        self.publisher.publish(
            PlpMessage(None, "Waypoint", "achieve", "PLP goal achieved"))
        self.reset_harness_data()

    def plp_no_preconditions(self):
        """ Called when the PLP is active and would have given an estimation,
            except that the PLP preconditions have not been met. """
        self.publisher.publish(PlpMessage(None, "Waypoint", "info", "PLP triggered, but its preconditions have not been met (yet)"))

    def plp_missing_data(self):
        """ Called by the PLP when it should have delivered an estimation,
            but there is not enough data (e.g. map is missing). """
        self.publisher.publish(PlpMessage(None, "Waypoint", "info", "PLP triggered, but its missing some data"))

    def plp_monitor_message(self, message):
        self.publisher.publish(
            PlpMessage(None, "Waypoint", "monitor",
                        repr(message)))


if __name__ == '__main__':
    try:
        rospy.loginfo("Starting plp_waypoint node...")
        harness = PlpWaypointRosHarness()

        rospy.loginfo("started")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
