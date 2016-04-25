from PlpIbeoClasses import PlpIbeoVariables

class PlpIbeo(object):
    """The PLP for an IBEO unit. """
    def __init__(self, constants, callback):
        super(PlpIbeo, self).__init__()
        self.arg = arg
        self.vars = PlpIbeoVariables(constants["RAY_COUNT"])

    def parameters_updated(self, arg):
        """Listener method. Called by a harness, when the parameters are updated.
           Triggers a detection and a possible callback."""
        # TODO implement
        pass

    def detect(self, arg):
        """Method where detection occures. Returns an array of warnings"""
        # TODO implement
        pass
