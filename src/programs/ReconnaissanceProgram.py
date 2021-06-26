import math
from struct import unpack
from typing import List, Tuple

import pygame
from paho.mqtt.client import MQTTMessage, Client
from bresenham import bresenham

from src.programs.IProgram import Command, IProgram, Measurement


class Grid:
    def __init__(self, block_size_m: float):
        self.block_size_m = block_size_m
        self.grid = {}

    def on_cell_not_seen(self, x, y):
        cell = self.get_cell(x, y)
        self.set_cell(x, y, max(cell - 1, 0))

    def on_cell_seen(self, x, y):
        cell = self.get_cell(x, y)
        self.set_cell(x, y, min(cell + 1, 9))

    def set_cell(self, x, y, val):
        if y not in self.grid:
            self.grid[y] = {}

        self.grid[y][x] = val

    def get_cell(self, x, y):
        if y in self.grid and x in self.grid[y]:
            return self.grid[y][x]

        return -1

    def print(self, center_x, center_y, range_x, range_y, path=None):
        print('\n\n')

        min_x = center_x - range_x
        max_x = center_x + range_x

        min_y = center_y - range_y
        max_y = center_y + range_y

        for y in range(min_y, max_y):
            line = ''

            for x in range(min_x, max_x):
                line += ' '
                if self.get_cell(x, y) == -1:
                    line += ' '
                    continue
                if self.get_cell(x, y) > 0:
                    line += str(self.grid[y][x])
                else:
                    line += ' '

            print(line)


class ReconnaissanceProgram(IProgram):
    def __init__(self, mqtt_client: Client, screen):
        super().__init__(mqtt_client, screen)
        self.robot_x = 0
        self.robot_y = 0
        self.robot_rot = 0
        self.last_command: Command = Command(0, 'r', 0)
        self.grid = Grid(0.75)

    def estimate_bot_position(self, progress: float):
        if progress == 0.0:
            return self.robot_x, self.robot_y, self.robot_rot

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
                print(math.degrees(self.robot_rot), self.robot_x, self.robot_y)

            path_to_target = self.find_closest_unknown()
            if path_to_target is not None:
                if len(path_to_target) == 2:
                    next_position = path_to_target[0]
                else:
                    next_position = path_to_target[-2]

                difference = (next_position[0] - self.robot_x, next_position[1] - self.robot_y)
                direction = math.atan2(difference[0], difference[1])
                direction_deg = math.degrees(direction)
                distance = math.sqrt(difference[0] * difference[0] + difference[1] * difference[1])

                if self.robot_rot != direction:
                    rotation = direction - self.robot_rot
                    rotation_deg = math.degrees(rotation)
                    self.last_command = Command(1, 'r', rotation)
                    self.send_command(robot_id, self.last_command)
                    return

                self.last_command = Command(2, 'f', distance)
                self.send_command(robot_id, self.last_command)

    def distance(self, a, b):
        difference = (a[0] - b[0], a[1] - b[1])
        return math.sqrt(difference[0] * difference[0] + difference[1] * difference[1])

    def find_closest_unknown(self) -> List[Tuple]:
        unexplored = []
        explored = {}

        search_x = int(self.robot_x / self.grid.block_size_m)
        search_y = int(self.robot_y / self.grid.block_size_m)

        start = {'position': (search_x, search_y), 'parent': None, 'distance': 0.0}
        unexplored.append(start)
        explored[(search_x, search_y)] = start

        while len(unexplored) > 0:
            unexplored.sort(key=lambda x: x['distance'], reverse=True)
            exploring = unexplored.pop()
            coordinate = (exploring['position'][0], exploring['position'][1])

            for rel_x in range(-1, 2):
                for rel_y in range(-1, 2):
                    if rel_x == 0 and rel_y == 0:
                        continue

                    abs_x = coordinate[0] + rel_x
                    abs_y = coordinate[1] + rel_y

                    if self.grid.get_cell(abs_x, abs_y) == -1:  # found something to explore
                        path = [(abs_x, abs_y), coordinate]

                        parent_position = exploring['parent']
                        while parent_position is not None:
                            path.append(parent_position)
                            parent_position = explored[parent_position]['parent']

                        return path

                    if self.grid.get_cell(abs_x, abs_y) == 0 and (abs_x, abs_y) not in explored:
                        new = {
                            'position': (abs_x, abs_y),
                            'parent': coordinate,
                            'distance': self.distance((abs_x, abs_y), (search_x, search_y))
                        }
                        explored[(abs_x, abs_y)] = new
                        unexplored.append(new)

        return None

    def on_sensor_data(self, robot_id: int, data: List[Measurement]):
        s = pygame.Surface((1000, 1000))
        s.set_alpha(1)
        s.fill((0, 0, 0))
        self.screen.blit(s, (0, 0))

        for measurement in data:
            if measurement.command_id == self.last_command.command_id:
                if measurement.distance == 0:
                    measurement.distance = 8

                est_x, est_y, est_rot = self.estimate_bot_position(measurement.progress)
                rot = est_rot + measurement.servo
                ray_x = est_x + math.sin(rot) * measurement.distance
                ray_y = est_y + math.cos(rot) * measurement.distance

                grid_est_x = int(round(est_x / self.grid.block_size_m))
                grid_est_y = int(round(est_y / self.grid.block_size_m))
                grid_ray_x = int(round(ray_x / self.grid.block_size_m))
                grid_ray_y = int(round(ray_y / self.grid.block_size_m))

                pygame.draw.line(
                    self.screen,
                    color=(255, 0, 255),
                    start_pos=(500+est_x*100, 500+est_y*100),
                    end_pos=(500+ray_x*100, 500+ray_y*100)
                )

                clear_cells = list(
                    bresenham(
                        grid_est_x,
                        grid_est_y,
                        int(round(est_x + math.sin(rot) * (measurement.distance - self.grid.block_size_m))),
                        int(round(est_y + math.cos(rot) * (measurement.distance - self.grid.block_size_m)))
                    )
                )

                for clear_cell in clear_cells:
                    if measurement.distance < 8 and clear_cell[0] == grid_ray_x and clear_cell[1] == grid_ray_y:
                        break

                    self.grid.on_cell_not_seen(clear_cell[0], clear_cell[1])

                if measurement.distance < 8:
                    self.grid.on_cell_seen(grid_ray_x, grid_ray_y)

        # self.grid.print(0, 0, 10, 10)
        pygame.display.flip()
