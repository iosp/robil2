from math import sqrt
from itertools import izip, imap, chain
from PlpAchieveResult import *
from PlpWaypointData import *


class PlpWaypoint(object):
    """
    Off-line calculation of the success probability of
    making it to the next waypoint.
    The PLP also emits progress monitoring messages.
    """

    def __init__(self, constant_map, start_data, callback):
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

        # Inputs
        self.map_error = start_data.map_error
        self.map = start_data.map
        self.path = start_data.path
        self.position = start_data.position
        self.position_error = start_data.position_error

        # variables
        self.map_occupancy = None
        self.local_path_distance = None
        self.height_variability = None
        self.aerial_distance = None

    def request_estimation(self):
        """
        Manually trigger estimation attempt.
        Typical client code use is from the harness,
        immediately after instantiating the PLP object.
        """
        if self.can_estimate():
            res = self.get_estimation()
            if res != None:
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
        result.success = (1-self.map_occupancy)*pow(3, -0.01*self.local_path_distance)
        result.success_time = (1+self.map_occupancy)*self.local_path_distance*self.constants["BOBCAT_AVERAGE_SPEED"]
        result.side_effects["fuel"] = self.local_path_distance \
                                      * self.constants["FUEL_CONSUMPTION_RATE"] \
                                      * self.height_variability * 1.7
        result.add_failure(PlpAchieveResultFailureScenario("bobcat_stuck",
                                                           sqrt(self.map_occupancy),
                                                           self.map_occupancy * self.local_path_distance
                                                           * self.constants["BOBCAT_AVERAGE_SPEED"]))
        result.add_failure(PlpAchieveResultFailureScenario("bobcat_turned_over",
                                                           0.005 * sqrt(self.map_occupancy),
                                                           0.5 * self.map_occupancy * self.local_path_distance
                                                           * pow(self.constants["BOBCAT_AVERAGE_SPEED"], 2)))
        result.confidence = 0.7

        return result

    def can_estimate(self):
        """
        Checks to see if this object has enough data calculate the estimation
        :return: True iff there is enough data; False otherwise.
        """
        return not((self.map is None)
                   or (self.path is None)
                   or (self.position is None)
                   or (self.position_error is None))

    def validate_preconditions(self):
        return self.local_path_distance > 1 and self.constants["MIN_LOC_ERROR"] > self.location_error_in_meters()

    def calculate_variables(self):
        """
        Initializes the PLP variables (see docs) based on the current state of the inputs.
        :return: None - changes are made on the object's state.
        """
        self.map_occupancy = self.calc_map_occupancy()
        self.height_variability = self.calc_map_height_variability()
        self.local_path_distance = self.calc_local_path_distance()
        self.aerial_distance = self.calc_aerial_distance()

    def calc_map_occupancy(self):
        """
        Calculates the map occupancy. If no cells were scanned, returns 0.
        :return:  (# vacant_cells)/(# scanned_cells)
        """
        occupied_cells = 0
        vacant_cells = 0
        for cell in self.map.data:
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
        for cell in self.map.data:
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
        local_planned_path = map(lambda p: p.pose, self.path.waypoints.poses)
        local_actual_path = chain([self.position], local_planned_path)
        pairs = izip(local_actual_path, local_planned_path)
        dist_between_points = imap(PlpWaypoint.dist_between_tuple, pairs)

        return sum(dist_between_points)

    def calc_aerial_distance(self):
        destPos = self.path.waypoints.poses[len(self.path.waypoints.poses)-1]
        self.dist_between(self.position, destPos.pose)

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

    # Updaters ##############################

    def set_map(self, a_map):
        self.map = a_map

    def set_position(self, a_position):
        self.position = a_position.pose.pose
        self.position_error = a_position.pose.covariance

    def set_path(self, a_path):
        self.path = a_path
