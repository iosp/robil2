"""Data the PLP needs, as defined in the PLP document.
Classes in this module should be generated automatically from the plp.xml,
once we get to it.
"""

class PlpWaypointParameters(object):
    """ Stores the parameterd for the waypoint PLP."""

    def __init__(self):
        self.map = None
        self.path = None
        self.position = None
        self.position_error = None
        self.map_error = 0 # Currently there's no error.
                          # When visual location will be added, there will be.

    def set_map(self, a_map):
        self.map = a_map

    def set_position(self, a_position):
        self.position = a_position.pose.pose
        self.position_error = a_position.pose.covariance

    def set_path(self, a_path):
        self.path = a_path


class PlpWaypointVariables(object):
    """Stores the variables (as defined in the PLP doc)"""

    def __init__(self):
        self.distanceToWaypoint = None
        self.mapOccupancy = None
        self.heightVariablity = None
        self.aerialDistanceToWaypoint = None
