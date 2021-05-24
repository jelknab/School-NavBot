import paho.mqtt.client as mqtt
import sys


def on_message(client, userdata, msg):
    print("received message: ", str(msg.payload.decode("utf-8")))


if __name__ == '__main__':
    client = mqtt.Client("controller")
    client.on_message = on_message

    client.username_pw_set(sys.argv[1], sys.argv[2])
    client.connect('localhost')
    client.subscribe('sensors/#')

    client.loop_forever()
