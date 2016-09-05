from math import sqrt
from itertools import izip, imap, chain
from PlpAchieveClasses import *
from PlpWaypointClasses import *

# Number of frames of variable history needed for monitoring.
PLP_WAYPOINT_HISTORY_LENGTH = 2


# TODO: Make sure variables are calculated at most once per parameter update.
# TODO: Also ensure var history does not update once per variable calculation
# TODO: Add progress measure timer to the PLP Harness.
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
        :param parameters: parameters object for this PLP. Not copied! used by reference.
        :param callback: progress monitoring and PLP status calls are sent to this object.
        :return: a new PlpWaypoint object
        """
        # constants and fields
        self.constants = constant_map
        self.callback = callback
        self.parameters = parameters
        self.variables_history = list()

    def request_estimation(self):
        """
        Manually trigger estimation attempt.
        Typical client code use is from the harness,
        immediately after instantiating the PLP object.
        Another use case is when estimating a plan, offline.
        """
        if self.can_estimate():
            res = self.get_estimation()
            if res is not None:
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

    def detect_termination(self):
        """
        See if any of the termination conditions applies.
        :return: A PLPTermination object, or None.
        """
        res = self.detect_success()
        if not ( res is None ):
            return res
        res = self.detect_failures()
        if not ( res is None ):
            return res
        return None

    def can_estimate(self):
        """
        Checks to see if this object has enough data calculate the estimation
        :return: True iff there is enough data; False otherwise.
        """
        return not ((self.parameters.map is None)
                    or (self.parameters.path is None)
                    or (self.parameters.position is None)
                    or (self.parameters.position_error is None))

    def validate_preconditions(self):
        return self.variables().local_path_distance > 1 \
               and self.constants["MIN_LOC_ERROR"] > self.location_error_in_meters()

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
        result = PlpAchieveResult()
        result.success = self.estimate_success()
        result.success_time = self.estimate_success_time()
        result.side_effects["fuel"] = self.estimate_fuel_side_effect()
        result.add_failure( self.estimate_bobcat_stuck_failure() )
        result.add_failure( self.estimate_bobcat_turned_over_failure() )
        result.confidence = self.get_estimation_confidence()
        return result

    def estimate_success(self):
        return (1 - self.variables().map_occupancy) \
                         * pow(3, -0.01 * self.variables().local_path_distance)

    def estimate_success_time(self):
        return (1 + self.variables().map_occupancy) \
                              * self.variables().local_path_distance \
                              * self.constants["BOBCAT_AVERAGE_SPEED"]

    def estimate_fuel_side_effect(self):
        return self.variables().local_path_distance \
                                      * self.constants["FUEL_CONSUMPTION_RATE"] \
                                      * self.variables().height_variability * 1.7

    def estimate_bobcat_stuck_failure(self) :
        current_vars = self.variables()
        probability = sqrt(current_vars.map_occupancy)
        time = current_vars.map_occupancy * current_vars.local_path_distance * self.constants["BOBCAT_AVERAGE_SPEED"]
        return PlpAchieveResultFailureScenario("bobcat_stuck", probability, time)

    def estimate_bobcat_turned_over_failure(self) :
        current_vars = self.variables()
        probability = 0.005 * sqrt(current_vars.map_occupancy)
        time = 0.5 * current_vars.map_occupancy * \
                current_vars.local_path_distance * \
                pow(self.constants["BOBCAT_AVERAGE_SPEED"], 2)
        return PlpAchieveResultFailureScenario("bobcat_turned_over", probability, time)

    def get_estimation_confidence(self):
        return 0.7 + self.constants["BOBCAT_AVERAGE_SPEED"]/10000

    #
    # Methods for detecting termination conditions ###############################

    def detect_success(self):
        if self.variables().aerial_distance < self.constants["GOAL_DISTANCE"]:
            return PlpTermination(True, "Arrived at goal")
        else:
            return None

    def detect_failures(self):
        return None  # Currently, not supported

    #
    # Advancement measurements ###################################################

    # Hard coded
    def monitor_progress(self):
        self.monitor_remaining_path_length()
        self.monitor_distance_to_target()

    def monitor_remaining_path_length(self):
        if len(self.variables_history) > 1:
            cur = self.variables_history[0].local_path_distance
            prv = self.variables_history[1].local_path_distance
            expected = prv * self.constants["RATE_PATH_LENGTH"]
            is_ok = expected >= cur
            msg = "" if is_ok else ("path length should be <= %s" % expected)
            self.callback.plp_monitor_message(
                PlpMonitorMessage("Path Length Monitor", is_ok, msg))

    def monitor_distance_to_target(self):
        if len(self.variables_history) > 1:
            cur = self.variables_history[0].aerial_distance
            prv = self.variables_history[1].aerial_distance
            expected = prv * self.constants["RATE_AERIAL_DISTANCE"]
            is_ok = expected >= cur
            msg = "" if is_ok else ("distance should be <= %s" % expected)
            self.callback.plp_monitor_message(
                PlpMonitorMessage("Distance to Target Monitor", is_ok, msg))

    #
    #  Methods to calculate the PLP variables #######################

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
        else:
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

        return float(vacant_cells) / float(vacant_cells + occupied_cells)

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
        average = total / float(len(values))
        diffs = map(lambda x: pow(x - average, 2), values)
        total_diff = sum(diffs)
        average_diff = total_diff / len(diffs)
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
        dest_pos = self.parameters.path.waypoints.poses[len(self.parameters.path.waypoints.poses) - 1]
        return self.dist_between(self.parameters.position, dest_pos.pose)

    #
    # Updaters ##############################

    def parameters_updated(self):
        """
        Called when parameters are updated.
        Can trigger monitoring and/or estimation
        """
        self.calculate_variables()
        termination = self.detect_termination()
        if termination is None:
            self.request_estimation()
            self.monitor_progress()
        else:
            self.callback.plp_terminated(termination)

    #
    # Utility methods ##############################

    def location_error_in_meters(self):
        x_error = self.parameters.position_error[0]
        y_error = self.parameters.position_error[6 + 1]
        return sqrt(pow(x_error, 2) + pow(y_error, 2))

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
        return sqrt(pow(point_a.position.x - point_b.position.x, 2) +
                    pow(point_a.position.y - point_b.position.y, 2) +
                    pow(point_a.position.z - point_b.position.z, 2))

    @staticmethod
    def constants_map():
        return {
            "MIN_LOC_ERROR": 5,  # meters
            "BOBCAT_SIZE": (3.5, 2, 1.7),  # LxHxW, meters
            "MIN_BLADE_CLEARANCE": 1,  # meters
            "FUEL_CONSUMPTION_RATE": 10000,  # m/liter
            "BOBCAT_AVERAGE_SPEED": 20000,  # m/hour
            "RATE_PATH_LENGTH": 0.85,  # 0..1
            "RATE_AERIAL_DISTANCE": 0.95,  # 0..1
            "GOAL_DISTANCE": 5  # Meters from target to be considered a success.
        }
