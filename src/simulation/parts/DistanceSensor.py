import math
from struct import pack
from typing import Tuple, List

from shapely.geometry import LineString

from src.simulation.ISimulating import ISimulating
from src.simulation.parts.Bot import Bot, Command
from src.simulation.parts.Servo import Servo
import paho.mqtt.client as mqtt


class DistanceSample:
    def __init__(self, bot: Bot, servo_rotation: float, distance_mm: float):
        self.position = (bot.x, bot.y)
        self.progress = bot.command.progress
        self.command_id = bot.command.id
        self.rotation = bot.rotation
        self.servo_rotation = servo_rotation
        self.distance_mm = distance_mm

    def as_array(self):
        return [self.command_id, self.progress, self.rotation, self.servo_rotation, self.distance_mm]


class DistanceSensor(ISimulating):
    def __init__(self,
                 min_distance_mm: float,
                 max_distance_mm: float,
                 sample_frequency_hz: float,
                 room: [Tuple[Tuple[float, float], Tuple[float, float]]],
                 servo: Servo):
        self.min_distance_mm = min_distance_mm
        self.max_distance_mm = max_distance_mm
        self.sample_frequency_hz = sample_frequency_hz
        self.speed_seconds = 1.0 / sample_frequency_hz
        self.room = room
        self.servo = servo
        self.sample_buffer: List[DistanceSample] = []

        self.mqtt_client = mqtt.Client("1_sensor")
        self.mqtt_client.connect('localhost', 10000)

    def send_sample_buffer(self):
        samples = pack('bffff'*len(self.sample_buffer), *[item for sample in self.sample_buffer for item in sample.as_array()])
        self.mqtt_client.publish('sensors/1', samples)

    def find_distance(self, bot: Bot, servo_rotation: float):
        distances = []

        measure_point = (
            bot.x + math.sin(servo_rotation + bot.rotation) * self.max_distance_mm,
            bot.y + math.cos(servo_rotation + bot.rotation) * self.max_distance_mm
        )

        detection_line = LineString([(bot.x, bot.y), measure_point])
        for wall in self.room:
            intersection = detection_line.intersection(LineString([(wall[0][0], wall[0][1]), (wall[1][0], wall[1][1])]))
            if intersection:
                distances.append(math.hypot(intersection.x - bot.x, intersection.y - bot.y))

        if len(distances) > 0:
            return min(distances)

        return 0

    def simulate(self, bot: Bot, seconds_delta: float, seconds_passed: float):
        if len(self.sample_buffer) > self.sample_frequency_hz:
            self.send_sample_buffer()
            self.sample_buffer.clear()

        times_executed_before = math.floor((seconds_passed - seconds_delta) / self.speed_seconds)
        times_executed_after = math.floor(seconds_passed / self.speed_seconds)
        times_executed = times_executed_after - times_executed_before
        last_execution = times_executed_before * self.speed_seconds

        for measurement_index in range(times_executed):
            measurement_execution_time = last_execution + self.speed_seconds * (measurement_index + 1)
            servo_rotation = self.servo.get_rotation_at_time(measurement_execution_time)
            distance = self.find_distance(bot, servo_rotation)
            self.sample_buffer.append(DistanceSample(bot, servo_rotation, distance))

