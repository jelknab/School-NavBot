from simulation.ISimulating import ISimulating


class DistanceSensor(ISimulating):
    def __init__(self, min_distance_cm: float, max_distance_cm: float):
        self.min_distance_cm = min_distance_cm
        self.max_distance_cm = max_distance_cm

    def simulate(self, seconds_delta: float, seconds_passed: float):
        pass
