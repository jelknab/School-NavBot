import math

from src.simulation.ISimulating import ISimulating


class Servo(ISimulating):
    def __init__(self, degrees_per_second: float):
        self.degrees_per_second = degrees_per_second
        self.rotation = 0.0

    def get_rotation_at_time(self, seconds_passed):
        return abs((math.radians(seconds_passed / self.degrees_per_second) % 6.28319) - 3.14159)

    def simulate(self, bot, seconds_delta: float, seconds_passed: float):
        self.rotation = self.get_rotation_at_time(seconds_passed)
