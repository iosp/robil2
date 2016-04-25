
class PlpIbeoVariables(object):
    """Variables calculated by the IBEO PLP, for its detection goal."""
    def __init__(self, ray_names):
        super(PlpIbeoVariables, self).__init__()
        self.last_scan_time = 0
        self.scan_time_increment = 0
        self.fail_or_cover_pcnt = emptyRayDictionary(ray_names)
        self.obstacle_pcnt = emptyRayDictionary(ray_names)
        self.sky_pcnt = emptyRayDictionary(ray_names)

    @staticmethod
    def emptyRayDictionary(ray_names):
        """Generate an array of Nones at the given length"""
        ret_val = {}
        for r in ray_names:
            ret_val[r]=None
        return ret_val

class DetectionMessage(object):
    """Message about a detected condition"""
    def __init__(self, key, actual_value, threshold, message):
        super(DetectionMessage, self).__init__()
        self.key = key
        self.actual_value = actual_value
        self.threshold = threshold
        self.message = message

    def __repr__(self):
        return "[DetectionMessage key:%s actual_value:%s threshold:%s message:%s]" % (self.key, self.actual_value, self.threshold, self.message)
