import paho.mqtt.client as mqtt
import threading
from flask import Flask
from flask_cors import CORS

import pygame

from programs.ReconnaissanceProgram import ReconnaissanceProgram

server = Flask(__name__)
CORS(server)

pygame.init()
screen = pygame.display.set_mode([1000, 1000])

client = mqtt.Client("controller")
program = ReconnaissanceProgram(client, screen)


@server.route("/")
def hello_world():
    grid = program.grid.grid
    min_y, max_y, min_x, max_x = None, None, None, None

    for y in grid.keys():
        for x in grid[y].keys():
            if min_y is None or y < min_y:
                min_y = y

            if max_y is None or y > max_y:
                max_y = y

            if min_x is None or x < min_x:
                min_x = x

            if max_x is None or x > max_x:
                max_x = x

    if min_y is None or max_y is None or min_x is None or max_x is None:
        return 'no grid boi'

    out = ''
    for y in range(min_y, max_x):
        row = []
        for x in range(min_x, max_x):
            if program.robot_x // program.grid.block_size_m == x and program.robot_y // program.grid.block_size_m == y:
                row.append(str(10))
                continue

            if y in grid and x in grid[y]:
                row.append(str(grid[y][x]))
            else:
                row.append('null')

        out += ", ".join(row)
        out += '; '

    return out


def on_message(client: mqtt.Client, userdata, msg: mqtt.MQTTMessage):
    program.on_message(msg)


def flash_run():
    server.run("0.0.0.0", 5000, False)


if __name__ == '__main__':
    threading.Thread(target=flash_run).start()
    client.on_message = on_message

    # client.username_pw_set(sys.argv[1], sys.argv[2])
    client.connect('localhost', 10000)
    client.subscribe([('sensors/#', 0), ('robots/#', 0)])

    client.loop_start()

    run = True
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
