from array import array
from struct import pack
from typing import List

from paho.mqtt.client import MQTTMessage, Client

import struct


class Measurement:
    def __init__(self, command_id: int, progress: float, rotation: float, servo_rotation: float, distance: float):
        self.command_id = command_id
        self.distance = distance
        self.servo = servo_rotation
        self.progress = progress
        self.bearing = rotation


class IProgram:
    def send_command(self, bot_id: int, command_id: int, command: str, value: float):
        command = pack('bcf', command_id, command.encode('ascii'), value)
        self.mqtt_client.publish(f'navigation/{bot_id}', command)

    def on_message(self, msg: MQTTMessage):
        topic_parts = msg.topic.split('/')
        topic_topic = topic_parts[0]
        topic_robot_id = int(topic_parts[1])

        self.topics.get(topic_topic, 'Unknown topic')(topic_robot_id, msg)

    def decode_robot_data(self, robot_id: int, msg: MQTTMessage):
        item_size = struct.calcsize('bffff')
        item_count = len(msg.payload) // item_size
        data = struct.unpack('bffff'*item_count, msg.payload)

        data_container: List[Measurement] = []

        for index in range(item_count):
            data_container.append(Measurement(
                data[index * 5 + 0],
                data[index * 5 + 1],
                data[index * 5 + 2],
                data[index * 5 + 3],
                data[index * 5 + 4]
            ))

        self.on_sensor_data(robot_id, data_container)

    def on_robot_data(self, robot_id: int, msg: MQTTMessage):
        raise NotImplementedError

    def on_sensor_data(self, robot_id: int, data: List[Measurement]):
        raise NotImplementedError

    def __init__(self, mqtt_client: Client):
        self.mqtt_client = mqtt_client
        self.topics = {
            'sensors': self.decode_robot_data,
            'robots': self.on_robot_data
        }
