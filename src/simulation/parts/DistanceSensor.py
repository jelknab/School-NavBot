import math
from typing import Tuple, List

from shapely.geometry import LineString

from src.simulation.ISimulating import ISimulating
from src.simulation.parts.Body import Body
from src.simulation.parts.Servo import Servo


class DistanceSample:
    def __init__(self, position: Tuple[float, float], servo_rotation: float, distance_mm: float):
        self.position = position
        self.servo_rotation = servo_rotation
        self.distance_mm = distance_mm


class DistanceSensor(ISimulating):
    def __init__(self,
                 min_distance_mm: float,
                 max_distance_mm: float,
                 sample_frequency_hz: float,
                 room: [Tuple[Tuple[float, float], Tuple[float, float]]],
                 servo: Servo):
        self.min_distance_mm = min_distance_mm
        self.max_distance_mm = max_distance_mm
        self.speed_seconds = 1.0 / sample_frequency_hz
        self.room = room
        self.servo = servo
        self.sample_buffer: List[DistanceSample] = []

    def find_distance(self, bot_position: Tuple[float, float], servo_rotation: float):
        distances = []

        measure_point = (
            bot_position[0] + math.sin(servo_rotation) * self.max_distance_mm,
            bot_position[1] + math.cos(servo_rotation) * self.max_distance_mm
        )

        detection_line = LineString([bot_position, measure_point])
        for wall in self.room:
            intersection = detection_line.intersection(LineString([(wall[0][0], wall[0][1]), (wall[1][0], wall[1][1])]))
            if intersection:
                distances.append(math.hypot(intersection.x - bot_position[0], intersection.y - bot_position[1]))

        if len(distances) > 0:
            return min(distances)

        return 0

    def simulate(self, bot: Body, seconds_delta: float, seconds_passed: float):
        self.sample_buffer.clear()

        times_executed_before = math.floor((seconds_passed - seconds_delta) / self.speed_seconds)
        times_executed_after = math.floor(seconds_passed / self.speed_seconds)
        times_executed = times_executed_after - times_executed_before
        last_execution = times_executed_before * self.speed_seconds

        for measurement_index in range(times_executed):
            measurement_execution_time = last_execution + self.speed_seconds * (measurement_index + 1)
            servo_rotation = self.servo.get_rotation_at_time(measurement_execution_time)
            distance = self.find_distance((bot.x, bot.y), servo_rotation)
            self.sample_buffer.append(DistanceSample((bot.x, bot.y), servo_rotation, distance))

    def read_and_clear_samples(self) -> List[DistanceSample]:
        return self.sample_buffer[:]

