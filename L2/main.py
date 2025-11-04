# IMPORTS
import numpy as np
import paho.mqtt.client as mqtt
import time
import json

# CONSTANTS
# Image size
X = 640
Y = 480

# Field of view
H_FOV = 87
V_FOV = 58

# Parameters for control
REG_CHOICE = '' # TWO-POS/PID
DT = 0.01 # s
# Parameters for two-post control
HORIZONTAL_ANGLE_THRESHOLD = 0.087 # rad
FACING_DISTANCE_THRESHOLD = 0.1
# Parameters for PID control
SAT_TH_ANGLE = 0.1
SAT_TH_DISTANCE = 0.1
# Distance PID
KP_ANGLE = 0.1
KI_ANGLE = 0
KD_ANGLE = 0
# Angle PID
KP_DIST = 0.1
KI_DIST = 0
KD_DIST = 0

# PARAMETERS FOR MQTT COMMUNICATION
MQTT_PUBLISH_TOPIC = "robot/cmd_vel"    # output
# sending [vx, vy, omega] array containing translational (x,y) and rotatational velocities
# in SI units (m/s, m/s, rad/s)
MQTT_SUBSCRIBE_TOPIC = "robot/cmd_vel_1"    # input
# receiving distance from detected object (distance) and it's center placement
# on the image (C)
MQTT_BROKER_IP = "192.168.50.53"

# CLASSES
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def compute(self, process_variable, dt):
            error = self.setpoint - process_variable
            P_out = self.Kp * error
            self.integral += error * dt
            I_out = self.Ki * self.integral
            derivative = (error - self.previous_error) / dt
            D_out = self.Kd * derivative
            output = P_out + I_out + D_out
            self.previous_error = error
            
            return output

# FUNCTIONS
def move_forawrd_saturation(value):
    if value > SAT_TH_DISTANCE:
        result = SAT_TH_DISTANCE
    elif value < -SAT_TH_DISTANCE:
        result = -SAT_TH_DISTANCE
    else:
        result = value
    return result

def rotate_saturation(value):
    if value > SAT_TH_ANGLE:
        result = SAT_TH_ANGLE
    elif value < -SAT_TH_ANGLE:
        result = -SAT_TH_ANGLE
    else:
        result = value
    return result

def mqtt_on_message(client, userdata, msg):
    global distance, Cx, Cy
    try:
        data = json.loads(msg.payload.decode())
        Cx = float(data.get("Cx",0))
        Cy = float(data.get("Cy",0))
        distance = float(data.get("distance",0))
    except:
        print("Error.")


# MAIN
# Globals/Inputs
Cx = 0
Cy = 0
distance = 0

# Setpoints
SP_distance = 0.3 # 30cm
SP_angle = 0

if(REG_CHOICE) == 'PID':
    PID_distance = PIDController(KP_DIST,KI_DIST,KD_DIST,SP_distance)
    PID_angle = PIDController(KP_ANGLE,KI_ANGLE,KD_ANGLE,SP_angle)
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.connect(MQTT_BROKER_IP, 1883, 60)
mqttc.on_message = mqtt_on_message
timer = time.time()
mqttc.loop_start()

# Loop
while 1:
    mqttc.subscribe(MQTT_SUBSCRIBE_TOPIC)
    C = [Cx, Cy]
    if time.time() - timer > DT: # timed loop
        timer = time.time()
        A = np.deg2rad((C[0] - X//2)/X*H_FOV) # horizontal_angle_difference
        #D = distance*np.cos(A)
        D = distance
        if REG_CHOICE == 'TWO-POS':
            rotate = 0
            if A > HORIZONTAL_ANGLE_THRESHOLD:
                rotate = SAT_TH_ANGLE
            elif A < -HORIZONTAL_ANGLE_THRESHOLD:
                rotate = -SAT_TH_ANGLE
            move_forward = 0
            if D > SP_distance + FACING_DISTANCE_THRESHOLD:
                move_forward = SAT_TH_DISTANCE
            elif D < SP_distance -FACING_DISTANCE_THRESHOLD:
                move_forward = -SAT_TH_DISTANCE
        elif REG_CHOICE == 'PID':
            Pmove_forward = PID_distance.compute(D, DT)
            rotate = PID_angle.compute(A, DT)
            move_forward = move_forawrd_saturation(move_forward)
            rotate = rotate_saturation(rotate)
        else:
            break
        out_data = {"vx":float(move_forward), "vy":0.0, "omega":float(rotate)}
        out_msg = json.dumps(out_data, separators=(',', ':'))
        mqttc.publish(MQTT_PUBLISH_TOPIC, out_msg, 0)
    
mqttc.loop_stop()