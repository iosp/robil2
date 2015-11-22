from math import sqrt
from itertools import izip, imap, chain
from PlpAchieveClasses import *
from PlpWaypointClasses import *

# Number of frames of variable history needed for monitoring.
PLP_WAYPOINT_HISTORY_LENGTH = 2

class PlpWaypoint(object):
    """
    Off-line calculation of the success probability of
    making it to the next waypoint.
    The PLP also emits progress monitoring messages.
    """

    def __init__(self, constant_map, parameters, callback):
        """
        :param constant_map: constants used in the calculation.
            See docs (in the docs folder).
        :param start_data: initial state for the fields
        :param callback: progress monitoring callbacks are send here
        :return: a new PlpWaypoint object
        """
        # constants and fields
        self.constants = constant_map
        self.callback = callback
        # We should sent estimation only once per request.
        self.estimation_sent = False

        # Input parameters
        self.parameters = parameters
        parameters.callback = self

        # variables
        self.variables_history = list()

    def request_estimation(self):
        """
        Manually trigger estimation attempt.
        Typical client code use is from the harness,
        immediately after instantiating the PLP object.
        """
        self.estimation_sent = False
        if self.can_estimate():
            res = self.get_estimation()
            if res != None:
                self.estimation_sent = True
                self.callback.plp_estimation(res)
            else:
                self.callback.plp_no_preconditions()
        else:
            self.callback.plp_missing_data()


    def get_estimation(self):
        """
        Generate an estimation iff the preconditions are met.
        Otherwise, return None.
        """
        self.calculate_variables()
        if self.validate_preconditions():
            return self.estimate()
        else:
            return None

    def estimate(self):
        """
        Estimates the probability of getting to the next point, and the side
        effects it may have. Assumes parameters have been calculated,
        preconditions have been met, etc.
        This is a "private" function, call get_estimation for the full
        preparation, validation and calculation.
        :return: An estimation of the success and side effects of
                 getting to the next point.
        """
        # TODO: This is a dummy implementation
        result = PlpAchieveResult()
        vars = self.variables()
        result.success = (1-vars.map_occupancy)*pow(3, -0.01*vars.local_path_distance)
        result.success_time = (1+vars.map_occupancy)*vars.local_path_distance*vars.constants["BOBCAT_AVERAGE_SPEED"]
        result.side_effects["fuel"] = vars.local_path_distance \
                                      * vars.constants["FUEL_CONSUMPTION_RATE"] \
                                      * vars.height_variability * 1.7
        result.add_failure(PlpAchieveResultFailureScenario("bobcat_stuck",
                                                           sqrt(vars.map_occupancy),
                                                           vars.map_occupancy * vars.local_path_distance
                                                           * vars.constants["BOBCAT_AVERAGE_SPEED"]))
        result.add_failure(PlpAchieveResultFailureScenario("bobcat_turned_over",
                                                           0.005 * sqrt(vars.map_occupancy),
                                                           0.5 * vars.map_occupancy * vars.local_path_distance
                                                           * pow(vars.constants["BOBCAT_AVERAGE_SPEED"], 2)))
        result.confidence = 0.7

        return result

    def can_estimate(self):
        """
        Checks to see if this object has enough data calculate the estimation
        :return: True iff there is enough data; False otherwise.
        """
        return not((self.parameters.map is None)
                   or (self.parameters.path is None)
                   or (self.parameters.position is None)
                   or (self.parameters.position_error is None))

    def validate_preconditions(self):
        return self.variables().local_path_distance > 1 and self.constants["MIN_LOC_ERROR"] > self.location_error_in_meters()


    # Monitoring ###################################################
    def monitor_progress(self):
        self.monitor_remaining_path_length()
        self.monitor_distance_to_target()

    def monitor_remaining_path_length(self):
        if len(self.variables_history) > 1:
            cur = self.variables_history[0].local_path_distance
            prv = self.variables_history[1].local_path_distance
            is_ok = prv*self.contants["RATE_PATH_LENGTH"] >= cur
            self.callback.plp_monitor_message( PlpMonitorMessage("Path Length Monitor", is_ok, ""))

    def monitor_distance_to_target(self):
        if len(self.variables_history) > 1:
            cur = self.variables_history[0].aerial_distance
            prv = self.variables_history[1].aerial_distance
            is_ok = prv*self.contants["RATE_AERIAL_DISTANCE"] >= cur
            self.callback.plp_monitor_message( PlpMonitorMessage("Distance to Target Monitor", is_ok, ""))


    # Methods to calculate the PLP variables #######################

    def calculate_variables(self):
        """
        Calculates the PLP variables (see docs) based on the current state of the inputs.
        Also manages the variable history.
        :return: None - changes are made on the object's state.
        """
        variables = PlpWaypointVariables()
        variables.map_occupancy = self.calc_map_occupancy()
        variables.height_variability = self.calc_map_height_variability()
        variables.local_path_distance = self.calc_local_path_distance()
        variables.aerial_distance = self.calc_aerial_distance()
        if len(self.variables_history) >= PLP_WAYPOINT_HISTORY_LENGTH:
            self.variables_history = [variables] + self.variables_history[0:-1]
        else
            self.variables_history = [variables] + self.variables_history

    def variables(self):
        """Returns the current variables"""
        return self.variables_history[0]

    def calc_map_occupancy(self):
        """
        Calculates the map occupancy. If no cells were scanned, returns 0.
        :return:  (# vacant_cells)/(# scanned_cells)
        """
        occupied_cells = 0
        vacant_cells = 0
        for cell in self.parameters.map.data:
            if cell.type == 1:
                vacant_cells += 1
            elif cell.type == 2:
                occupied_cells += 1

        if occupied_cells + vacant_cells == 0:
            return 0  # corner case, where no scans were made

        return float(vacant_cells)/float(vacant_cells+occupied_cells)

    def calc_map_height_variability(self):
        """
        :return:  the map height standard deviation.
        """
        # first, we collect the values for easy manipulation
        values = []
        for cell in self.parameters.map.data:
            if cell.type == 1:
                values.append(cell.height)

        # Average
        if len(values) == 0:
            return 1

        total = sum(values)
        average = total/ float(len(values))
        diffs = map(lambda x: pow(x-average, 2), values)
        total_diff = sum(diffs)
        average_diff = total_diff/len(diffs)
        return pow(average_diff, 0.5)

    def calc_local_path_distance(self):
        """
        :return: Euclidean distance to the next waypoint, in Meters.
        """
        local_planned_path = map(lambda p: p.pose, self.parameters.path.waypoints.poses)
        local_actual_path = chain([self.parameters.position], local_planned_path)
        pairs = izip(local_actual_path, local_planned_path)
        dist_between_points = imap(PlpWaypoint.dist_between_tuple, pairs)

        return sum(dist_between_points)

    def calc_aerial_distance(self):
        destPos = self.parameters.path.waypoints.poses[len(self.parameters.path.waypoints.poses)-1]
        self.dist_between(self.parameters.position, destPos.pose)


    # Updaters ##############################

    def parameters_updated(self) :
        """Called when parameters are updated.
           Can trigger monitoring and/or estimation"""
        if not self.estimation_sent:
            self.request_estimation()

        self.monitor_progress()


    # Utility methods ##############################

    def location_error_in_meters(self):
        x_error = self.position_error[0]
        y_error = self.position_error[6+1]
        return sqrt(pow(x_error, 2)+pow(y_error, 2))

    @staticmethod
    def dist_between_tuple(tpl):
        return PlpWaypoint.dist_between(tpl[0], tpl[1])

    @staticmethod
    def dist_between(point_a, point_b):
        """
        :param point_a:
        :param point_b:
        :return: Euclidean distance between point_a and point_b.
        """
        return sqrt(pow(point_a.position.x-point_b.position.x, 2)
                    + pow(point_a.position.y-point_b.position.y, 2)
                    + pow(point_a.position.z-point_b.position.z, 2))
