import math
import os
import time
from typing import List

import pygame
import paho.mqtt.client as mqtt
from shapely.geometry import LineString, Point

from src.simulation.parts.Body import Body
from src.simulation.parts.DistanceSensor import DistanceSensor, DistanceSample
from src.simulation.parts.Servo import Servo

os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (0, 30)

pygame.init()
screen = pygame.display.set_mode([600, 600])
camera_x = screen.get_width() / 2
camera_y = screen.get_height() / 2
display_scale = 0.04  # Pixel per mm

simulation_timestamp = time.time()
simulation_speed_s = .004  # seconds = 250hz

room = [
    ((-5000, -5000), (-5000, 10000)),
    ((-5000, 10000), (3000, 10000)),
    ((3000, 10000), (3000, 5000)),
    ((3000, 5000), (7000, 5000)),
    ((7000, 5000), (7000, -5000)),
    ((7000, -5000), (-5000, -5000))
]

robot = Body(0.25, 0.84)
servo = Servo(.17 / 60)
distance_sensor = DistanceSensor(200, 8000, 100, room, servo)


def camera(x, y):
    return x * display_scale + camera_x, y * display_scale + camera_y


def draw_servo(seconds_passed, total_seconds):
    pygame.draw.line(
        screen,
        color=(255, 0, 0),
        start_pos=camera(robot.x, robot.y),
        end_pos=camera(robot.x + math.sin(servo.rotation) * 1000, robot.y + math.cos(servo.rotation) * 1000)
    )


def draw_environment():
    for wall in room:
        pygame.draw.line(
            screen,
            color=(255, 255, 255),
            start_pos=camera(
                wall[0][0],
                wall[0][1]
            ),
            end_pos=camera(
                wall[1][0],
                wall[1][1]
            )
        )


def draw_distance_sensor(sample_buffer: List[DistanceSample]):
    for sample in sample_buffer:
        pygame.draw.line(
            screen,
            color=(255, 0, 255),
            start_pos=camera(sample.position[0], sample.position[1]),
            end_pos=camera(
                sample.position[0] + math.sin(sample.servo_rotation) * sample.distance_mm,
                sample.position[1] + math.cos(sample.servo_rotation) * sample.distance_mm
            )
        )


def simulate(seconds_passed, total_seconds):
    # fade
    s = pygame.Surface((screen.get_width(), screen.get_height()))
    s.set_alpha(1)
    s.fill((0, 0, 0))
    screen.blit(s, (0, 0))

    robot.simulate(robot, seconds_passed, total_seconds)
    servo.simulate(robot, seconds_passed, total_seconds)
    distance_sensor.simulate(robot, seconds_passed, total_seconds)

    draw_environment()
    draw_servo(seconds_passed, total_seconds)
    draw_distance_sensor(distance_sensor.sample_buffer)

    pygame.display.flip()


if __name__ == '__main__':
    run = True
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        while time.time() - simulation_timestamp < simulation_speed_s:
            pass

        seconds_elapsed = time.time() - simulation_timestamp
        simulation_timestamp = time.time()
        simulate(seconds_elapsed, simulation_timestamp)
