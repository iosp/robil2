
class PlpAchieveResultFailureScenario:
    """
    A failure scenario, complete with its probability, estimated time to fail, and name.
    """
    def __init__(self, name, probability, time):
        self.name = name
        self.probability = probability
        self.time = time


class PlpAchieveResult:
    """
    A result of an estimation of a PLP achieve module.
    Fields are:
    success: probability ([0..1])
    success_time: Time to achieve success
    confidence: How sure are we about the success estimation
    side_effects: Dictionary from string to the side effect (e.g. {'fuel':-18})
    failure: Dictionary of string to PlpAchieveResultFailScenario.
    """
    def __init__(self):
        self.success = None
        self.success_time = None
        self.confidence = None
        self.side_effects = {}
        self.failure = {}

    def add_failure(self, fail):
        self.failure[fail.name] = fail
