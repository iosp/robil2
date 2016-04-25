
class PlpIbeoVariables(object):
    """Variables calculated by the IBEO PLP, for its detection goal."""
    def __init__(self, rayCount):
        super(PlpIbeoVariables, self).__init__()
        self.last_scan_time = 0
        self.fail_or_cover_pcnt = arrayOfNones(rayCount)
        self.obstacle_pcnt = arrayOfNones(rayCount)
        self.sky_pcnt = arrayOfNones(rayCount)

    @staticmethod
    def arrayOfNones(len):
        """Generate an array of Nones at the given length"""
        retVal = []
        for i in range(0,len):
            retVal.append( None )
        return retVal
