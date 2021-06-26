import paho.mqtt.client as mqtt
import sys

import pygame

from programs.ReconnaissanceProgram import ReconnaissanceProgram


pygame.init()
screen = pygame.display.set_mode([1000, 1000])

client = mqtt.Client("controller")
program = ReconnaissanceProgram(client, screen)


def on_message(client: mqtt.Client, userdata, msg: mqtt.MQTTMessage):
    program.on_message(msg)


if __name__ == '__main__':
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
