#!/usr/bin/env python
import re
import rospy
import sys

from std_msgs.msg import String, Header  # TODO replace with the plp message
from PlpIbeoClasses import *
from PlpIbeo import *
from config import config
from plp_waypoint.msg import PlpMessage
from robil_msgs.msg import MultiLaserScan

PLP_TOPIC = "/plp/messages"

class PlpIbeoRosHarness(object):
    """
    The ROS harness for the IBEO PLP.
    """
    def __init__(self, constants):
        super(PlpIbeoRosHarness, self).__init__()
        self.plp_constants = constants
        # Init the ROS stuff
        rospy.init_node("plp_ibeo", anonymous=False)
        self.publisher = rospy.Publisher(PLP_TOPIC, PlpMessage, queue_size=5)
        rospy.Subscriber("/SENSORS/IBEO/1", MultiLaserScan, self.ibeoScan)

    def trigger_detection_start(self):
        """ Creates the PLP object, which starts the detection process. """
        self.plp = PlpIbeo(self.plp_constants, self)
        self.timer = rospy.Timer( rospy.Duration(self.plp_constants["WATCHDOG_PERIOD"]), self.scan_frequency_test, False)

    def ibeoScan(self, newMultiLaserScanData):
        if ( not (self.plp is None) ):
            self.plp.parameters_updated(newMultiLaserScanData, rospy.get_time())

    def scan_frequency_test(self, timer_event):
        """Test that the last scan is not late"""
        detection = self.plp.test_scan_frequency( rospy.get_time() )
        if not (detection is None):
            self.condition_detected(detection)


    def condition_detected(self, detection_message):
        self.publisher.publish(
            PlpMessage(None, "IBEO", "detect",
                       repr(detection_message)))

# Main entry point for the IBEO PLP ROS node.
if __name__ == '__main__':
    try:
        rospy.loginfo("Starting plp_ibeo node...")
        start_args = rospy.myargv(argv=sys.argv)

        harness = PlpIbeoRosHarness( config )
        rospy.loginfo("started")
        # The detection start on this PLP is system startup, so we start it here.
        harness.trigger_detection_start()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
