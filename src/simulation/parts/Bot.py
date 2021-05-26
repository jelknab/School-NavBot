import math
import time
from struct import *

import paho.mqtt.client as mqtt

from src.Bot import BotCommand
from src.simulation.ISimulating import ISimulating


class Command(BotCommand):
    def __init__(self, command_id: int, command: str, value: float, time_seconds: float):
        super().__init__(command_id, command, value)
        self.timestamp = time_seconds
        self.progress = 0

    def time_elapsed(self, time_passed):
        return time_passed - self.timestamp


class Bot(ISimulating):
    def __init__(self, speed_change_m_s: float, max_speed_m_s: float):
        self.x = 0
        self.y = 0
        self.id = 1
        self.speed = 0
        self.rotation = math.pi / 2 * 0
        self.command: Command = Command(-1, '', 0, 0)
        self.speed_ms = max_speed_m_s
        self.accel_ms = speed_change_m_s

        self.last_message_timestamp = 0
        self.mqtt_client = mqtt.Client("1")
        self.mqtt_client.connect('localhost')
        self.mqtt_client.on_message = self.receive_command
        self.mqtt_client.subscribe('navigation/1/#')
        self.mqtt_client.loop_start()

    def receive_command(self, client: mqtt.Client, userdata, msg: mqtt.MQTTMessage):
        command_id, command, value = unpack('bcf', msg.payload)
        command = command.decode('ascii')
        self.command = Command(command_id, command, value, time.time())
        pass

    def request_command(self):
        self.mqtt_client.publish('robots/1', pack('bb', 0x00, self.command.id))

    def progress_full_acceleration(self, t):
        # some constants for restricting relative times
        acceleration_duration = self.speed_ms / self.accel_ms
        acceleration_distance = 0.5 * self.accel_ms * acceleration_duration ** 2

        constant_distance = self.command.value - acceleration_distance * 2
        constant_duration = constant_distance / self.speed_ms

        # relative times for parts of the curve
        acceleration_t = min(t, acceleration_duration)
        constant_t = max(0, min(t - acceleration_duration, constant_duration))
        deceleration_t = max(0, min(t - acceleration_duration - constant_duration, acceleration_duration))

        acceleration_distance = 0.5 * self.accel_ms * acceleration_t ** 2
        constant_distance = self.speed_ms * constant_t
        deceleration_distance = self.speed_ms * deceleration_t - self.accel_ms / 2 * deceleration_t ** 2

        return acceleration_distance + constant_distance + deceleration_distance

    def progress_partial_acceleration(self, t):
        acceleration_distance = self.command.value / 2
        acceleration_duration = math.sqrt((2 * acceleration_distance) / self.accel_ms)
        acceleration_top_speed = self.accel_ms * acceleration_duration

        acceleration_t = min(t, acceleration_duration)
        deceleration_t = min(min(max(t - acceleration_duration, 0), acceleration_duration), acceleration_duration*2)

        acceleration_distance = 0.5 * self.accel_ms * acceleration_t ** 2
        deceleration_distance = acceleration_top_speed * deceleration_t - self.accel_ms / 2 * deceleration_t ** 2

        return acceleration_distance + deceleration_distance

    def simulate(self, bot, seconds_delta: float, seconds_passed: float):
        if self.command.progress == self.command.value and seconds_passed - self.last_message_timestamp > 5:
            self.last_message_timestamp = seconds_passed
            self.request_command()
            pass

        if self.command.command == 'f':
            t = self.command.time_elapsed(seconds_passed)

            accel_duration = self.speed_ms / self.accel_ms
            accel_distance = 0.5 * self.accel_ms * accel_duration ** 2

            old_progress = self.command.progress

            if self.command.value / 2 > accel_distance:
                self.command.progress = self.progress_full_acceleration(t)
            else:
                self.command.progress = self.progress_partial_acceleration(t)

            difference = self.command.progress - old_progress

            self.x += math.sin(self.rotation) * difference * 1000
            self.y += math.cos(self.rotation) * difference * 1000
