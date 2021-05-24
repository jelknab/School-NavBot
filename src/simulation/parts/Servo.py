from simulation.ISimulating import ISimulating


class Servo(ISimulating):
    def __init__(self, min_rotation: float, max_rotation: float, degrees_per_second: float):
        self.min_rotation = min_rotation
        self.max_rotation = max_rotation
        self.degrees_per_second = degrees_per_second
        self.rotation = 0
        self.target_rotation = 0

    def simulate(self, seconds_delta: float, seconds_passed: float):
        pass

    def set_rotation(self, rotation: float):
        self.target_rotation = rotation
