"""Module for the PlpWaypointData class"""

class PlpWaypointData(object):
    """ Stores all the data needed by the waypoint PLP.
    Can be auto-generated completely by the code gen, when we get to it.
    """

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
