# IMPORTS
import numpy as np
import paho.mqtt.client as mqtt
import time
import json

MQTT_PUBLISH_TOPIC = "robot/reg_input"    # output
MQTT_BROKER_IP = "localhost"

mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.connect(MQTT_BROKER_IP, 1883, 60)
mqttc.loop_start()

timer = time.time()
DT = 0.01 

while 1:
    #if time.time() - timer > DT:
        #timer = time.time()
        Cx = input("Give Cx:")
        Cy = input("Give Cy:")
        distance = input("Give distance:")
        out_data = {"Cx":float(Cx), "Cy":float(Cy), "distance":float(distance)}
        out_msg = json.dumps(out_data, separators=(',', ':'))
        mqttc.publish(MQTT_PUBLISH_TOPIC, out_msg, 0)

mqttc.loop_stop()
