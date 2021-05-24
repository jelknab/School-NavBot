import math
import os
import time
import pygame
import paho.mqtt.client as mqtt
from shapely.geometry import LineString, Point

os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (0, 30)

pygame.init()
screen = pygame.display.set_mode([600, 600])
camera_x = screen.get_width() / 2
camera_y = screen.get_height() / 2
display_scale = 0.02  # Pixel per mm

simulation_timestamp = time.time()
simulation_speed_s = .001  # seconds = 100hz

robot_x = 0.0
robot_y = 0.0
robot_yaw = 0.0

servo_rotation_degree_per_second = .17 / 60  # 0.0027 seconds per degree ~.5 second for 180deg
servo_precision = 180 / (2000 / 5)  # total rotation / (timing period / dead bandwidth)
servo_current_yaw = 0.0

distance_sensor_speed = 0.01  # 0.01 seconds = 100hz, 0.004 seconds = 250hz
distance_sensor_min_distance = 200  # mm
distance_sensor_max_distance = 8000  # mm

room = [
    ((-5000, -5000), (-5000, 10000)),
    ((-5000, 10000), (3000, 10000)),
    ((3000, 10000), (3000, 5000)),
    ((3000, 5000), (7000, 5000)),
    ((7000, 5000), (7000, -5000)),
    ((7000, -5000), (-5000, -5000))
]


def camera(x, y):
    return x * display_scale + camera_x, y * display_scale + camera_y


def get_servo_rotation(time):
    return abs((math.radians(time / servo_rotation_degree_per_second) % 6.28319) - 3.14159)


def simulate_servo(seconds_passed, total_seconds):
    servo_target_rotation = get_servo_rotation(total_seconds)

    servo_direction_x = math.sin(servo_target_rotation)
    servo_direction_y = math.cos(servo_target_rotation)

    pygame.draw.line(
        screen,
        color=(255, 0, 0),
        start_pos=camera(robot_x, robot_y),
        end_pos=camera(robot_x + servo_direction_x * 1000, robot_y + servo_direction_y * 1000)
    )


def find_distance(start_point, end_point):
    distances = []

    line1 = LineString([start_point, end_point])
    for wall in room:
        intersection = line1.intersection(LineString([(wall[0][0], wall[0][1]), (wall[1][0], wall[1][1])]))
        if intersection:
            distances.append(math.hypot(intersection.x - robot_x, intersection.y - robot_y))

    if len(distances) > 0:
        return min(distances)

    return 0


def simulate_distance_sensor(seconds_passed, total_seconds):
    times_executed_before = math.floor((total_seconds - seconds_passed) / distance_sensor_speed)
    times_executed_after = math.floor(total_seconds / distance_sensor_speed)
    times_executed = times_executed_after - times_executed_before
    last_execution = times_executed_before * distance_sensor_speed

    for measurement_index in range(0, times_executed):
        measurement_execution_time = last_execution + distance_sensor_speed * (measurement_index + 1)
        servo_yaw = get_servo_rotation(measurement_execution_time)

        start_point = (robot_x, robot_y)
        end_point = (
            robot_x + math.sin(servo_yaw) * distance_sensor_max_distance,
            robot_y + math.cos(servo_yaw) * distance_sensor_max_distance
        )

        distance = find_distance(start_point, end_point)

        pygame.draw.line(
            screen,
            color=(255, 0, 255),
            start_pos=camera(robot_x, robot_y),
            end_pos=camera(
                robot_x + math.sin(servo_yaw) * distance,
                robot_y + math.cos(servo_yaw) * distance
            )
        )


def simulate(seconds_passed, total_seconds):
    global robot_x, robot_y

    s = pygame.Surface((screen.get_width(), screen.get_height()))
    s.set_alpha(1)
    s.fill((0, 0, 0))
    screen.blit(s, (0, 0))

    robot_x = -5000 + (total_seconds * 1000 % 10000)

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

    simulate_servo(seconds_passed, total_seconds)
    simulate_distance_sensor(seconds_passed, total_seconds)
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
