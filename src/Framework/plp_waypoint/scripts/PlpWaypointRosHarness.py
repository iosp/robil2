#!/usr/bin/env python
# always
import rospy
import sys
import pickle

# From Glue
from robil_msgs.msg import Map, Path, AssignNavTask, AssignMission
from geometry_msgs.msg import PoseWithCovarianceStamped

# Because PLP messages
from std_msgs.msg import String, Header  # TODO replace with the plp message
from plp_waypoint.msg import PlpMessage

# Imported generated classes
from PlpWaypoint import *
from PlpWaypointClasses import *

# glue (output topic)
PLP_TOPIC = "/plp/messages"


class PlpWaypointRosHarness(object):
    """ A harness for a PlpWaypoint in a ROS/RobIL system. Listens to the
        right topics, feeds the data to the PLP objects, and emits predictions
        when possible."""

    def __init__(self):
        # The constants table is defined in the PLP document
        # under "Values/Constants"

        self.plp_constants = PlpWaypoint.constants_map()

        self.node_setup()

        # Setup internal PLP objects.
        self.plp = None
        self.plp_params = PlpWaypointParameters()

        # Parse arguments. decide on trigger mode: Capture, Run PLP, or both.
        self.capture = False
        self.monitor = False
        self.capture_filename = ""
        start_args = rospy.myargv(argv=sys.argv)
        for arg in start_args[1:]:
            print(arg)
            if arg == "-capture":
                self.capture = True
            elif arg == "-monitor":
                self.monitor = True
            elif self.capture:
                self.capture_filename = arg

        # default: just use the PLP.
        if not (self.capture or self.monitor):
            self.monitor = True

        if self.capture and self.capture_filename == "":
            self.capture_filename = "capture-file"

        # Init the ROS stuff
        rospy.init_node("plp_waypoint", anonymous=False)
        self.publisher = rospy.Publisher(PLP_TOPIC, PlpMessage, queue_size=5)
        rospy.Subscriber("/PER/MiniMap", Map, self.map_updated)
        rospy.Subscriber("/PP/Path", Path, self.path_updated)
        rospy.Subscriber("/LOC/Pose", PoseWithCovarianceStamped,
                         self.position_updated)
        rospy.Subscriber("/OCU/SMME/NavigationTask", AssignNavTask,
                         self.nav_task_assigned)
        rospy.Subscriber("/OCU/SMME/MissionPlan", AssignMission,
                         self.mission_assigned)
        rospy.Subscriber("/decision_making/events", String,
                         self.state_machine_change)

        if self.monitor:
            rospy.loginfo("Trigger action: Monitoring")
        if self.capture:
            rospy.loginfo("Trigger action: Capturing (filename: %s)" % self.capture_filename)

        rospy.loginfo("Waypoint PLP Harness - Started")

    def node_setup(self):
        """ Put your custom node initialization code here
        """
        self.nav_tasks = {}  # id -> nav_task
        self.missions = {}  # id -> mission
        self.mission_state = {}  # mission_id -> current task index.
        self.trigger_nav_task_active = False
        self.trigger_local_path_published = False

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
        """ This method has to do with trigger detection. We wait for the
            mission to be "spooling" - ROBIL-speak for "active", really -
            and then we can have a "go" on the trigger w.r.t. a nav
            task being active.
        """
        comps = event_string.data.split("/")
        # Test for the triggering of a task.
        if (len(comps) == 8 and
                    comps[1] == "mission" and
                    comps[3] == "TaskActive" and
                    comps[5] == "TaskSpooling"):
            mission_id = comps[2]
            self.activate_mission(mission_id)

        elif len(comps) == 4 and comps[1] == "mission":
            mission_id = comps[2]
            event = comps[3]
            if event in {"AbortMission", "CompleteMission", "ClearMission"}:
                if self.mission_state.has_key(mission_id):
                    del self.mission_state[mission_id]
                    self.reset_harness_data()
            elif event == "StartMission":
                self.activate_mission(mission_id)

    def activate_mission(self, mission_id):
        if self.mission_state.has_key(mission_id):
            self.mission_state[mission_id] += 1
        else:
            self.mission_state[mission_id] = 0

        task_index = self.mission_state[mission_id]

        # test if the task is a navigation task
        rospy.loginfo(
            "Task #{0} of mission {1} spooling".format(task_index, mission_id))
        if self.missions.has_key(mission_id):
            mission = self.missions[mission_id]
            # task_id = mission.tasks[task_index].task_id
            # if self.nav_tasks.has_key(task_id):
            # TODO validate that the task is a navigation task. Former method
            #      based on indices does not work. Mayeb listen to a new topic?
            rospy.loginfo("Will attempt calculation once local path is ready")
            self.trigger_local_path_published = False
            self.trigger_nav_task_active = True

    def consider_trigger(self):
        """
        Test the status of the fields. If all preconditions are met,
        trigger the plp.
        """
        if self.trigger_nav_task_active and self.trigger_local_path_published:
            self.trigger_local_path_published = False
            self.trigger_nav_task_active = False
            if self.monitor:
                self.trigger_plp_task()
            if self.capture:
                self.capture_params()

    def trigger_plp_task(self):
        """
        Creates a PLP and starts the monitoring, if there's no PLP yet.
        """
        rospy.loginfo("Activating PLP")
        self.plp = PlpWaypoint(self.plp_constants, self.plp_params, self)
        self.plp_params.callback = self.plp
        self.plp.request_estimation()

    def reset_harness_data(self):
        self.plp = None
        self.plp_params.callback = None
        self.plp_params = PlpWaypointParameters()
        self.trigger_local_path_published = False
        self.trigger_nav_task_active = False

    # PLP Callback methods #####################################
    def plp_estimation(self, plp_achieve_result):
        """ The PLP is active, and gives an estimation.
        :param  plp_achieve_result: the result estimation from the PLP.
        """
        self.publisher.publish(
            PlpMessage(None, "Waypoint", "estimation",
                       repr(plp_achieve_result)))

    def plp_terminated(self, plp_termination):
        """
        The PLP detected that one of its termination conditions have occurred.
        Deletes the current PLP, resets the harness.
        :param plp_termination: The termination message sent from the PLP.
        """
        rospy.loginfo("PLP terminated")

        if plp_termination.is_success():
            msg = "PLP goal achieved"
        else:
            msg = "PLP goal achievement failed: " + plp_termination.get_message()

        self.publisher.publish(
            PlpMessage(None, "Waypoint", "achieve", msg))
        self.reset_harness_data()

    def plp_no_preconditions(self):
        """ Called when the PLP is active and would have given an estimation,
            except that the PLP preconditions have not been met. """
        self.publisher.publish(
            PlpMessage(None, "Waypoint", "info", "PLP triggered, but its preconditions have not been met (yet)"))

    def plp_missing_data(self):
        """ Called by the PLP when it should have delivered an estimation,
            but there is not enough data (e.g. map is missing). """
        self.publisher.publish(PlpMessage(None, "Waypoint", "info", "PLP triggered, but its missing some data"))

    def plp_monitor_message(self, message):
        self.publisher.publish(
            PlpMessage(None, "Waypoint", "monitor",
                       repr(message)))

    #
    # Capture parameters at trigger ############################
    def capture_params(self):
        capture_file = open(self.capture_filename, "w")
        pickle.dump(self.plp_params, capture_file)
        capture_file.close()

        rospy.loginfo("Captured parameters at trigger time to file '%s'" % self.capture_filename)


if __name__ == '__main__':
    try:
        rospy.loginfo("Starting plp_waypoint node...")
        harness = PlpWaypointRosHarness()

        rospy.loginfo("started")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
