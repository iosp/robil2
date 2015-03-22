from robil_msgs.msg import Map.msg


class PlpWaypointEstimation:
    """
    Probabilities of getting to the next waypoint, the time it will take, and the confidence level.
    This is a record/"value class" - no logic, just named values.
    """
    def __init__(self, success, time, confidence):
        self.success = success
        self.time = time
        self.confidence = confidence


class PlpWaypoint:
    """
    Off-line calculation of the success probability of making it to the next waypoint.
    """

    def __init__(self, constant_map):
        """
        :param constant_map: constants used in the calculation. See docs (in the docs folder).
        :return: a new PlpWaypoint object
        """
        # constants
        self.constants = constant_map

        # Inputs
        self.map_error = 0  # Currently there's no error. When visual location will be added, there will be.
        self.map = None
        self.path = None
        self.position = None
        self.position_error = None

        # variables
        self.map_occupancy = None
        self.distance_to_waypoint = None
        self.height_variability = None

    def get_estimation(self):
        self.init_variables()

        # predict
        return PlpWaypointEstimation(1, 1, 1)

    def init_variables(self):
        """
        Initializes the PLP variables (see docs) based on the current state of the inputs.
        :return: None - changes are made on the object's state.
        """
        map_occ = calc_map_occupancy( self.map )

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
        vals = []
        for cell in self.map.data:
            if cell.type == 1:
                vals.append(cell.height)

        # Average
        total = sum(vals)
        average = total/float(len(vals))
        diffs = map(lambda x: pow(x-average, 2), vals)
        total_diff = sum(diffs)
        average_diff = total_diff/len(diffs)
        return pow(average_diff, 0.5)

    ### Updaters

    def update_map(self, a_map):
        self.map = a_map

    def update_position(self, a_position):
        self.position = a_position

    def update_path(self, a_path):
        self.path = a_path

    def update_position_error(self, a_position_error):
        self.position_error = a_position_error