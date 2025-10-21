import paho.mqtt.client as mqtt
import json
import time

SUBSCRIBE_TOPIC = "robot/cmd_vel"
PUBLISH_TOPIC = "robot/cmd_vel_1"

def mqtt_on_message(client, userdata, msg):
    global vx, vy, omega
    try:
        data = json.loads(msg.payload.decode())
        vx = float(data.get("vx",0))
        vy = float(data.get("vy",0))
        omega = float(data.get("omega",0))
        print(vx,vy,omega)
    except:
        print("E")

vx = 0
vy = 0
omega = 0

mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.connect("192.168.50.53", 1883, 60)
mqttc.on_message = mqtt_on_message
timer = time.time()
DT = 10

mqttc.loop_start()
while(1):
    mqttc.subscribe(SUBSCRIBE_TOPIC)
    out_data = {"vx":float(vx), "vy":1000, "omega":float(omega)}
    out_msg = json.dumps(out_data, separators=(',', ':'))
    mqttc.publish(PUBLISH_TOPIC, out_msg, 0)
    time.sleep(1)
mqttc.loop_stop()
