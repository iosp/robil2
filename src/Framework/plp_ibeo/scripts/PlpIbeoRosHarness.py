#!/usr/bin/env python
import rospy

from std_msgs.msg import String, Header  # TODO replace with the plp message
from PlpIbeoClasses import *
from PlpIbeo import *
from plp_waypoint.msg import PlpMessage
from robil_msgs.msg import MultiLaserScan

PLP_TOPIC = "/plp/messages"

class PlpIbeoRosHarness(object):
    """
    The ROS harness for the IBEO PLP.
    """
    def __init__(self):
        super(PlpIbeoRosHarness, self).__init__()
        self.plp_constants = {
            "TIME_INCREMENT_TOLERANCE" : 0.01, # seconds
            "TIME_INCREMENT": 0.08, # seconds
            "FAIL_OR_COVER_THRESHOLD": 0.1,  # ratio (percent/100)
            "OBSTACLE_THRESHOLD": 0.3, # ratio (percent/100)
            "SKY_THRESHOLD": 0.8, # ratio (percent/100)
            "FAIL_OR_COVER_DISTANCE": 1, # meter - distance under which we assume IBEO is blocked
            "OBSTACLE_DISTANCE": 3, # meter - distance under which we assume close obstacle exists
            "SKY_DISTANCE": 50, # meter - distance over which we assume IBEO is looking at the sky
            "RAYS":["t1","t2","b1","b2"] # Names of the rays in the message.
        }
        # Init the ROS stuff
        rospy.init_node("plp_ibeo", anonymous=False)
        self.publisher = rospy.Publisher(PLP_TOPIC, PlpMessage, queue_size=5)
        rospy.Subscriber("/SENSORS/IBEO/1", MultiLaserScan, self.ibeoScan)

    def trigger_detection_start(self):
        """ Creates the PLP object, which starts the detection process. """
        self.plp = PlpIbeo(self.plp_constants, self)

    def ibeoScan(self, newMultiLaserScanData):
        # TODO update the PLP object
        if ( not (self.plp is None) ):
            self.plp.parameters_updated(newMultiLaserScanData)

    def condition_detected(self, detection_message):
        self.publisher.publish(
            PlpMessage(None, "IBEO", "detect",
                       repr(detection_message)))


# Main entry point for the IBEO PLP ROS node.
if __name__ == '__main__':
    try:
        rospy.loginfo("Starting plp_ibeo node...")
        harness = PlpIbeoRosHarness()
        rospy.loginfo("started")
        # The detection start on this PLP is system startup, so we start it here.
        harness.trigger_detection_start()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
