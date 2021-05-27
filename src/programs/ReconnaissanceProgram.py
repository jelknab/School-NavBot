from struct import unpack
from typing import List
from paho.mqtt.client import MQTTMessage
from programs.IProgram import IProgram, Measurement


class ReconnaissanceProgram(IProgram):
    def on_robot_data(self, robot_id: int, msg: MQTTMessage):
        request_type, last_command_id = unpack('bb', msg.payload)

        if request_type == 0x00:
            print(f'Bot {robot_id} is bored, waiting for command.')
            if last_command_id == -1:
                self.send_command(robot_id, 1, 'f', 4)
            if last_command_id == 1:
                self.send_command(robot_id, 2, 'r', 90)
            if last_command_id == 2:
                self.send_command(robot_id, 1, 'f', 4)


        pass

    def on_sensor_data(self, robot_id: int, data: List[Measurement]):
        print('recv sensor data')
        pass
