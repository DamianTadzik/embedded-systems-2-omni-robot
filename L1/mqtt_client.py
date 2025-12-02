import json
import paho.mqtt.client as mqtt

mqtt_request = {"vx": 0.0, "vy": 0.0, "omega": 0.0}


def on_connect(client, userdata, flags, rc):
    client.subscribe("robot/cmd_vel")

def on_message(client, userdata, msg):
    global vx, vy, omega
    try:
        data = json.loads(msg.payload.decode())
        mqtt_request["vx"] = float(data.get("vx", 0))
        mqtt_request["vy"] = float(data.get("vy", 0))
        mqtt_request["omega"] = float(data.get("omega", 0))
    except:
        pass

def start_mqtt(broker="localhost", port=1883):
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port)
    client.loop_start()
    return client
