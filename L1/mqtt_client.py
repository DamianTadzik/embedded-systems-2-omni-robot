import json
import paho.mqtt.client as mqtt

vx = vy = omega = 0.0

def on_connect(client, userdata, flags, rc):
    client.subscribe("robot/cmd_vel")

def on_message(client, userdata, msg):
    global vx, vy, omega
    try:
        data = json.loads(msg.payload.decode())
        vx = float(data.get("vx", 0))
        vy = float(data.get("vy", 0))
        omega = float(data.get("omega", 0))
    except:
        pass

def start_mqtt(broker="localhost", port=1883):
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port)
    client.loop_start()
    return client
