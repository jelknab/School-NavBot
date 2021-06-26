from array import array
from struct import pack
from typing import List

from paho.mqtt.client import MQTTMessage, Client

import struct


class Command:
    def __init__(self, command_id: int, command: str, value: float):
        self.value = value
        self.command = command
        self.command_id = command_id

    def pack(self):
        return pack('bcf', self.command_id, self.command.encode('ascii'), self.value)


class Measurement:
    def __init__(self, command_id: int, progress: float, servo_rotation: float, distance: float):
        self.command_id = command_id
        self.distance = distance
        self.servo = servo_rotation
        self.progress = progress


class IProgram:
    def send_command(self, bot_id: int, command: Command):
        self.mqtt_client.publish(f'navigation/{bot_id}', command.pack())

    def on_message(self, msg: MQTTMessage):
        topic_parts = msg.topic.split('/')
        topic_topic = topic_parts[0]
        topic_robot_id = int(topic_parts[1])

        self.topics.get(topic_topic, 'Unknown topic')(topic_robot_id, msg)

    def decode_robot_data(self, robot_id: int, msg: MQTTMessage):
        item_size = struct.calcsize('bfff')
        item_count = len(msg.payload) // item_size
        data = struct.unpack('bfff' * item_count, msg.payload)

        data_container: List[Measurement] = []

        for index in range(item_count):
            measurement = Measurement(
                data[index * 4 + 0],
                data[index * 4 + 1],
                data[index * 4 + 2],
                data[index * 4 + 3]
            )
            data_container.append(measurement)

        self.on_sensor_data(robot_id, data_container)

    def on_robot_data(self, robot_id: int, msg: MQTTMessage):
        raise NotImplementedError

    def on_sensor_data(self, robot_id: int, data: List[Measurement]):
        raise NotImplementedError

    def __init__(self, mqtt_client: Client, screen):
        self.screen = screen
        self.mqtt_client = mqtt_client
        self.topics = {
            'sensors': self.decode_robot_data,
            'robots': self.on_robot_data
        }
