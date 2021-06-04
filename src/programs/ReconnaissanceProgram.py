import math
from struct import unpack
from typing import List
from paho.mqtt.client import MQTTMessage, Client
from bresenham import bresenham

from src.programs.IProgram import Command, IProgram, Measurement


class Grid:
    def __init__(self, block_size_m: float):
        self.block_size_m = block_size_m
        self.grid = {}

    def set_grid_point(self, x, y, val):
        if y not in self.grid:
            self.grid[y] = {}

        self.grid[y][x] = val

    def print(self, center_x, center_y, range_x, range_y):
        print('\n\n')

        min_x = center_x - range_x
        max_x = center_x + range_x

        min_y = center_y - range_y
        max_y = center_y + range_y

        for y in range(min_y, max_y):
            line = ''

            for x in range(min_x, max_x):
                if y in self.grid and x in self.grid[y]:
                    line += self.grid[y][x]
                else:
                    line += ' '

            print(line)




class ReconnaissanceProgram(IProgram):
    def __init__(self, mqtt_client: Client):
        super().__init__(mqtt_client)
        self.robot_x = 0
        self.robot_y = 0
        self.robot_rot = 0
        self.last_command: Command = Command(-1, 'r', 0)
        self.grid = Grid(0.25)

    def estimate_bot_position(self, progress: float):
        if self.last_command.command == 'f':
            x = self.robot_x + math.sin(self.robot_rot) * progress
            y = self.robot_y + math.cos(self.robot_rot) * progress
            return x, y, self.robot_rot

        if self.last_command.command == 'r':
            return self.robot_x, self.robot_y, self.robot_rot + progress

        return self.robot_x, self.robot_y, self.robot_rot

    def on_robot_data(self, robot_id: int, msg: MQTTMessage):
        request_type, last_command_id = unpack('bb', msg.payload)

        if request_type == 0x00:
            print(f'Bot {robot_id} is bored, waiting for command.')

            if last_command_id == self.last_command.command_id:
                self.last_command.command_id = -1
                self.robot_x, self.robot_y, self.robot_rot = self.estimate_bot_position(self.last_command.value)

            if self.last_command.command == 'f':
                self.last_command = Command(1, 'r', math.pi / 2)
                self.send_command(robot_id, self.last_command)
                return

            if self.last_command.command == 'r':
                self.last_command = Command(2, 'f', 2)
                self.send_command(robot_id, self.last_command)
                return

    def on_sensor_data(self, robot_id: int, data: List[Measurement]):
        for measurement in data:
            if measurement.command_id == self.last_command.command_id:
                if measurement.distance > 0:
                    est_x, est_y, est_rot = self.estimate_bot_position(measurement.progress)
                    ray_x = est_x + math.sin(est_rot + measurement.servo) * measurement.distance
                    ray_y = est_y + math.cos(est_rot + measurement.servo) * measurement.distance

                    grid_est_x = int(round(est_x / self.grid.block_size_m))
                    grid_est_y = int(round(est_y / self.grid.block_size_m))
                    grid_ray_x = int(round(ray_x / self.grid.block_size_m))
                    grid_ray_y = int(round(ray_y / self.grid.block_size_m))

                    clear_cells = list(
                        bresenham(
                            grid_est_x,
                            grid_est_y,
                            grid_ray_x,
                            grid_ray_y
                        )
                    )

                    for clear_cell in clear_cells:
                        self.grid.set_grid_point(clear_cell[0], clear_cell[1], '.')

                    self.grid.set_grid_point(grid_ray_x, grid_ray_y, '#')

        self.grid.print(0, 0, 30, 30)
        pass
