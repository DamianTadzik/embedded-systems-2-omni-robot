# IMPORTS
import numpy as np
import paho.mqtt.client as mqtt
import time
import json

# CONSTANTS
# Image size
X = 1280
Y = 720

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
MQTT_TOPIC = "robot/cmd_vel"

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
    print(msg.topic+" "+str(msg.payload))

# INPUTS
distance = 1 # m, distance from the object
C = [900, 300] # [x,y] pixel coordinates, center of the object

# MAIN
# Setpoints
SP_distance = 0.3 # 30cm
SP_angle = 0
if(REG_CHOICE) == 'PID':
    PID_distance = PIDController(KP_DIST,KI_DIST,KD_DIST,SP_distance)
    PID_angle = PIDController(KP_ANGLE,KI_ANGLE,KD_ANGLE,SP_angle)
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_message = "Data sent."
mqttc.connect("localhost", 1883, 60)
timer = time.time()

# Loop
while(1):
    if time.time() - timer > DT: # timed loop
        timer = time.time()
        A = np.deg2rad((C[0] - X//2)/X*H_FOV) # horizontal_angle_difference
        D = distance*np.cos(A)
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
        mqttc.publish(MQTT_TOPIC, out_msg, 0)

# TO DO
# - check image constants (X,Y)
# - tune PID
