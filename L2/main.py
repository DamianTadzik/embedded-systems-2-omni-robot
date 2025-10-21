# IMPORTS
import numpy as np
import time

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
UPPER_SAT_TH = 1
LOWER_SAT_TH = 1
# Distance PID
KP_ANGLE = 1
KI_ANGLE = 0
KD_ANGLE = 0
# Angle PID
KP_DIST = 1
KI_DIST = 0
KD_DIST = 0

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

#FUNCTIONS
def saturation(value):
    if value > UPPER_SAT_TH:
        result = UPPER_SAT_TH
    elif value < LOWER_SAT_TH:
        result = LOWER_SAT_TH
    else:
        result = value
    return result

# INPUTS
distance = 1 # m, distance from the object
C = [900, 300] # [x,y] pixel coordinates, center of the object

# LOOP
# Setpoints
SP_distance = 0.3 # 30cm
SP_angle = 0
if(REG_CHOICE) == 'PID':
    PID_distance = PIDController(KP_DIST,KI_DIST,KD_DIST,SP_distance)
    PID_angle = PIDController(KP_ANGLE,KI_ANGLE,KD_ANGLE,SP_angle)
timer = time.time()

while(1):
    if time.time() - timer > DT: # pętla czasowa
        timer = time.time()
        A = np.deg2rad((C[0] - X//2)/X*H_FOV) # horizontal_angle_difference
        D = distance*np.cos(A)
        if REG_CHOICE == 'TWO-POS':
            rotate = 0
            if A > HORIZONTAL_ANGLE_THRESHOLD:
                rotate = 0.5
            elif A < -HORIZONTAL_ANGLE_THRESHOLD:
                rotate = -0.5
            move_forward = 0
            if D > SP_distance + FACING_DISTANCE_THRESHOLD:
                move_forward = 0.5
            elif D < SP_distance -FACING_DISTANCE_THRESHOLD:
                move_forward = -0.5
        elif REG_CHOICE == 'PID':
            Pmove_forward = PID_distance.compute(D, DT)
            rotate = PID_angle.compute(A, DT)
            move_forward = saturation(move_forward) # [-1,1] output
            rotate = saturation(rotate) # [-1,1] output
        else:
            break




