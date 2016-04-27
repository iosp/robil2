from itertools import *
from PlpIbeoClasses import *

class PlpIbeo(object):
    """The PLP for an IBEO unit. """
    def __init__(self, constants, callback):
        super(PlpIbeo, self).__init__()
        self.constants = constants
        self.vars = PlpIbeoVariables(constants["RAYS"])
        self.params = None
        self.callback = callback

    def parameters_updated(self, newScan):
        """Listener method. Called by a harness, when the parameters are updated.
           Triggers a detection and a possible callback."""
        self.params = newScan
        detections = self.detect()
        for d in detections:
            self.callback.condition_detected(d)

    def detect(self):
        """Method where detection occures. Returns an array of warnings"""
        self.calculate_variables()
        detections = []
        if self.vars.scan_time_increment>(self.constants["TIME_INCREMENT"]+self.constants["TIME_INCREMENT_TOLERANCE"]):
            detections.append( DetectionMessage("timeout",
                                                self.vars.scan_time_increment,
                                                self.constants["TIME_INCREMENT"],
                                                "IBEO scan was delayed"))
        for ray in self.constants["RAYS"]:
            if self.vars.fail_or_cover_pcnt[ray]>self.constants["FAIL_OR_COVER_THRESHOLD"]:
                detections.append( DetectionMessage("fail_or_cover@" + ray,
                                                    self.vars.fail_or_cover_pcnt[ray],
                                                    self.constants["FAIL_OR_COVER_THRESHOLD"],
                                                    "IBEO ray %s Seems to be covered or failing"%(ray)) )
            if self.vars.obstacle_pcnt[ray]>self.constants["OBSTACLE_THRESHOLD"]:
                detections.append( DetectionMessage("obstacle@" + ray,
                                                    self.vars.obstacle_pcnt[ray],
                                                    self.constants["OBSTACLE_THRESHOLD"],
                                                    "IBEO ray %s Seems to be facing an obstacle"%(ray)) )
            if self.vars.sky_pcnt[ray]>self.constants["SKY_THRESHOLD"]:
                detections.append( DetectionMessage("sky@" + ray,
                                                    self.vars.sky_pcnt[ray],
                                                    self.constants["SKY_THRESHOLD"],
                                                    "IBEO ray %s Seems to be looking at the sky"%(ray)) )
        return detections

    def calculate_variables(self):
        """Read the new params, update the variables"""
        vars = PlpIbeoVariables(self.constants["RAYS"])
        vars.scan_time_increment = self.params.time_increment
        vars.last_scan_time = self.params.scan_time
        for ray in self.constants["RAYS"]:
            vars.fail_or_cover_pcnt[ray] = self.calculate_fail_or_cover_pcnt( getattr(self.params, "ranges_" + ray) )
            vars.obstacle_pcnt[ray] = self.calculate_obstacle_pcnt( getattr(self.params, "ranges_" + ray) )
            vars.sky_pcnt[ray] = self.calculate_sky_pcnt( getattr(self.params, "ranges_" + ray) )
        # Store the now variables in the instance' field.
        self.vars = vars

    def calculate_sky_pcnt(self, ranges):
        count = len(ranges)
        if count == 0:
            return None
        sky_count =  sum(1 for _ in ifilter( lambda x: x>self.constants["SKY_DISTANCE"], ranges))
        return sky_count/count

    def calculate_obstacle_pcnt(self, ranges):
        count = len(ranges)
        if count == 0:
            return None
        obstacled_ranges = ifilter( lambda x: x<self.constants["OBSTACLE_DISTANCE"] and x>self.constants["FAIL_OR_COVER_DISTANCE"], ranges)
        obstacle_count =  sum(1 for _ in obstacled_ranges)
        return obstacle_count/count

    def calculate_fail_or_cover_pcnt(self, ranges):
        count = len(ranges)
        if count == 0:
            return None
        obstacle_count =  sum(1 for _ in ifilter( lambda x: x<self.constants["FAIL_OR_COVER_DISTANCE"], ranges))
        return obstacle_count/count
