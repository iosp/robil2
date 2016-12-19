"""Data the PLP needs, as defined in the PLP document.
Classes in this module should be generated automatically from the plp.xml,
once we get to it.
"""


class PlpWaypointParameters(object):
    """ Stores the parameters for the waypoint PLP."""

    def __init__(self):
        self.callback = None
        self.map = None
        self.path = None
        self.position = None
        self.position_error = None
        self.map_error = 0  # Currently there's no error. When visual location will be added, there will be.

    def set_map(self, a_map):
        self.map = a_map
        if self.callback:
            self.callback.parameters_updated()

    def set_position(self, a_position):
        self.position = a_position.pose.pose
        self.position_error = a_position.pose.covariance
        if self.callback:
            self.callback.parameters_updated()

    def set_path(self, a_path):
        self.path = a_path
        if self.callback: self.callback.parameters_updated()


class PlpWaypointVariables(object):
    """Stores the variables (as defined in the PLP doc)"""
    def __init__(self):
        self.local_path_distance = None
        self.map_occupancy = None
        self.height_variability = None
        self.aerial_distance = None


class PlpWaypointDevNullCallback(object):
    def plp_estimation(self, plp_achieve_result):
        pass

    def plp_terminated(self, plp_termination):
        pass

    def plp_no_preconditions(self):
        pass

    def plp_missing_data(self):
        pass

    def plp_monitor_message(self, message):
        pass