import paho.mqtt.client as mqtt
import sys

from programs.ReconnaissanceProgram import ReconnaissanceProgram

client = mqtt.Client("controller")
program = ReconnaissanceProgram(client)


def on_message(client: mqtt.Client, userdata, msg: mqtt.MQTTMessage):
    program.on_message(msg)


if __name__ == '__main__':
    client.on_message = on_message

    # client.username_pw_set(sys.argv[1], sys.argv[2])
    client.connect('localhost', 10000)
    client.subscribe([('sensors/#', 0), ('robots/#', 0)])

    client.loop_forever()
