from array import array
from struct import pack
from typing import List

from paho.mqtt.client import MQTTMessage, Client


class Measurement:
    def __init__(self, bearing: float, progress: float, servo: float, distance: float):
        self.distance = distance
        self.servo = servo
        self.progress = progress
        self.bearing = bearing


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
        data = array('f')
        data.frombytes(msg.payload)

        data_container: List[Measurement] = []

        for offset in range(len(data) // 4):
            data_container.append(
                Measurement(
                    data[offset * 4 + 0],
                    data[offset * 4 + 1],
                    data[offset * 4 + 2],
                    data[offset * 4 + 3],
                )
            )

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
