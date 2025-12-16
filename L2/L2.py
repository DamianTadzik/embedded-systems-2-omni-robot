# IMPORTS
import numpy as np
import paho.mqtt.client as mqtt
import time
import json
import threading

# CONSTANTS

## VISION SYSTEM CONSTANTS
### Image size
X = 640
Y = 480
### Field of view
H_FOV = 87
V_FOV = 58

## CONTROL CONSTANTS
### Controller choice (TWO-POS and PID available)
REG_CHOICE = 'TWO-POS' # TWO-POS/PID
### Detector timeout [s]
DETECTOR_TIMEOUT = 2
### Main loop cycle time
DT = 0.1 # s
### Parameters for TWO-POS control
TWO_POS_ANGLE_THRESHOLD = 0.087 # rad
TWO_POS_DISTANCE_THRESHOLD = 0.1
TWO_POS_DISTANCE_OUTPUT = 0.1
TWO_POS_ANGLE_OUTPUT = 0.1
### Parameters for PID control
PID_OUTPUT_TH_ANGLE = 0.5
PID_OUTPUT_TH_DISTANCE = 0.5
KP_ANGLE = 0.1
KI_ANGLE = 0
KD_ANGLE = 0
KP_DIST = 0.1
KI_DIST = 0
KD_DIST = 0

## MQTT CONSTANTS
### Publish topic ([vx, vy, omega] array of translational (X,Y) and rotatational velocities (Z) in SI)
MQTT_PUBLISH_TOPIC = "robot/cmd_vel"
## Subscribe topic ([dist, C] array of distance from the object and it's center placement in frame)
MQTT_SUBSCRIBE_TOPIC = "robot/nn_output"    # input
## MQTT broker IP
MQTT_BROKER_IP = "localhost"

## FILTERING CONSTANTS
### Filter choice
FILTER_CHOICE = "OSKAR" #ALPHA BETA NOT FUNCTIONAL- REQUIRES UPDATE
### Alpha Beta filter parameters
ANGLE_ALPHA = 0.4
ANGLE_BETA = 0.05
DISTANCE_ALPHA = 0.5
DISTANCE_BETA = 0.05
### Number od detections for startup
## 1 - TABULAR, 2 - CUTOFF, NONE
INPUT_FILTER_CHOICE = 'CUTOFF'
### Startup detections for tabular filter (1)
STARTUP_DETECTIONS = 20
### Std multiplier (1)
STD_MULT = 3
### Cutoff value (2)
INPUT_FILTER_CUTOFF = 2 # m

# LOGGING
### Logger on/off
LOGGER_ON_OFF = True

# CLASSES

## Custom error handles

class ControllerConfigError(Exception):
    pass

class FilterConfigError(Exception):
    pass

class InputFilterConfigError(Exception):
    pass

## Filtering

class AlphaBetaFilter:  # NOT FUNCTIONAL- REQUIRES UPDATE
    def __init__(self, alpha, beta, initial_value):
        self.alpha = alpha
        self.beta = beta
        self.x = initial_value
        self.v = 0.0
        self.last_update = time.time()

    def predict(self, dt):
        self.x -= self.v * dt

    def update(self, measurement, dt):
        r = measurement - self.x

        self.x += self.alpha * r
        self.v += (self.beta / dt) * r

    def step(self, measurement, new_detection_flag):
        now = time.time()
        dt = now - self.last_update
        self.last_update = now

        self.predict(dt)
        if new_detection_flag and measurement is not None:
            self.update(measurement, dt)

        return self.x

class OskarFilter:
    def __init__(self, initial_value):
        self.x = initial_value
        self.v = 0.0
        self.last_update = time.time()

    def predict(self, dt):
        self.x -= self.v * dt

    def update(self, measurement):
        self.x = measurement

    def step(self, measurement, new_detection_flag, v):
        now = time.time()
        dt = now - self.last_update
        self.last_update = now

        self.v = v
        self.predict(dt)
        if new_detection_flag and measurement is not None:
            self.update(measurement)

        return self.x


class FilterSetup:
    def __init__(self, filter_choice, initial_angle, initial_distance):
        self.filter_choice = filter_choice
        if filter_choice == 'ALPHA_BETA':
            self.angle_filter = AlphaBetaFilter(ANGLE_ALPHA, ANGLE_BETA, initial_angle)
            self.dist_filter = AlphaBetaFilter(DISTANCE_ALPHA, DISTANCE_BETA, initial_distance)
        elif filter_choice == 'OSKAR':
            self.angle_filter = OskarFilter(initial_angle)
            self.dist_filter = OskarFilter(initial_distance)
        else:
            raise FilterConfigError("Improper filter choice")

    def step(self, measurement_angle, measurement_dist, new_detection_flag, vx, vy, omega):
        if self.filter_choice == 'ALPHA_BETA':
            est_angle = self.angle_filter.step(measurement_angle, new_detection_flag)
            est_dist = self.dist_filter.step(measurement_dist, new_detection_flag)
        elif self.filter_choice == 'OSKAR':
            est_angle = self.angle_filter.step(measurement_angle, new_detection_flag, omega)
            est_dist = self.dist_filter.step(measurement_dist, new_detection_flag,
                                             vx/np.cos(est_angle))  # projection
        else:
            raise FilterConfigError("Improper filter choice")
        return est_angle, est_dist

class InputFilterByDistance:
    def __init__(self, length):
        self.detections = [None] * length
        self.tail = 0

    def add(self, val):
        self.detections[self.tail] = val
        if self.tail == STARTUP_DETECTIONS - 1:
            self.tail = 0
        else:
            self.tail += 1

    def verify(self, val):
        mean = np.mean(self.detections)
        std = np.std(self.detections)
        if np.abs(mean - val) > STD_MULT*std:
            return False
        else:
            self.add(val)
            return True

    def ready(self):
        return all(v is not None for v in self.detections)

class InputFilterNoFilter:
    def add(self, val):
        pass

    def verify(self, val):
        return True

    def ready(self):
        return True

class InputFilterCutoff:
    def add(self, val):
        pass

    def verify(self, val):
        if val < INPUT_FILTER_CUTOFF:
            return True
        else:
            return False

    def ready(self):
        return True

class InputFilterSetup:
    def __init__(self, input_filter_choice):
        if input_filter_choice == 'TABULAR':
            self.input_filter = InputFilterByDistance(STARTUP_DETECTIONS)
        elif input_filter_choice == 'CUTOFF':
            self.input_filter = InputFilterCutoff()
        elif input_filter_choice == 'NONE':
            self.input_filter = InputFilterNoFilter()
        else:
            raise InputFilterConfigError("Improper input filter choice")

    def add(self, val):
        self.input_filter.add(val)

    def verify(self, val):
        return self.input_filter.verify(val)

    def ready(self):
        return self.input_filter.ready()

## Controllers

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, threshold):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.threshold = threshold
        self.previous_error = 0
        self.integral = 0

    def saturation(self, val):
        result = val
        if result > self.threshold:
            result = self.threshold
        elif result < -self.threshold:
            result = -self.threshold
        return result

    def compute(self, var, dt):
        error = self.setpoint - var
        P_out = self.Kp * error
        self.integral += error * dt
        I_out = self.Ki * self.integral
        derivative = (error - self.previous_error) / dt
        D_out = self.Kd * derivative
        output = P_out + I_out + D_out
        self.previous_error = error
        return self.saturation(output)


class TwoPosCotroller:
    def __init__(self, setpoint, threshold, output_val):
        self.threshold = threshold
        self.output_val = output_val
        self.setpoint = setpoint

    def compute(self, var, dt):
        output = var
        if var - self.setpoint > self.threshold:
            output = self.output_val
        elif var - self.setpoint < -self.threshold:
            output = -self.output_val
        return output

class ControlSetup:
    def __init__(self, controller_choice, sp_angle, sp_distance):
        if controller_choice == 'TWO-POS':
            self.angle_controller = TwoPosCotroller(sp_angle, TWO_POS_ANGLE_THRESHOLD,
                                                 TWO_POS_ANGLE_OUTPUT)
            self.dist_controller = TwoPosCotroller(sp_distance, TWO_POS_DISTANCE_THRESHOLD,
                                                TWO_POS_DISTANCE_OUTPUT)
        elif controller_choice == 'PID':
            self.angle_controller = PIDController(KP_ANGLE, KI_ANGLE, KD_ANGLE, sp_angle,
                                                  PID_OUTPUT_TH_ANGLE)
            self.dist_controller = PIDController(KP_DIST, KI_DIST, KD_DIST, sp_distance,
                                                 PID_OUTPUT_TH_DISTANCE)
        else:
            raise ControllerConfigError("Improper controller choice")

    def compute(self, dist, angle, dt):
        vx = self.dist_controller.compute(dist, dt)
        omega = self.angle_controller.compute(angle, dt)
        return vx, 0.0, omega


## MQTT communication

class MQTTInterface:
    def __init__(self, mqtt_on_message_callback):
        self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqttc.connect(MQTT_BROKER_IP, 1883, 60)
        self.mqttc.on_message = mqtt_on_message_callback
        self.mqttc.loop_start()
        self.mqttc.subscribe(MQTT_SUBSCRIBE_TOPIC)

    def publish(self, out_data):
        out_msg = json.dumps(out_data, separators=(',', ':'))
        self.mqttc.publish(MQTT_PUBLISH_TOPIC, out_msg, 0)

## Control algorithm


class Follower:
    def __init__(self, sp_angle, sp_distance, controller_choice, filter_choice, logger_active):
        self.lock = threading.Lock()
        self.filter_choice = filter_choice
        self.controller_choice = controller_choice
        self.sp_angle = sp_angle
        self.sp_distance = sp_distance
        self.detector_active = False
        self.last_detection_time = 0
        self.filter = None
        self.controller = None
        self.mqtt = MQTTInterface(self.mqtt_callback)
        self.logger_active = logger_active
        if logger_active:
            self.log_file = open("logs.txt", "a")
            self.log_file.write("#" * 100 + "\n")
            self.log_file.write("Program started.\n")
            self.log_file.write("#" * 100 + "\n\n\n")

        self.raw_angle = None
        self.raw_dist = None

        self.last_rejected_distance = None

        self.new_detection = False
        self.input_filter = InputFilterSetup(INPUT_FILTER_CHOICE)
        self.startup_done = False


        self.vx = 0
        self.vy = 0
        self.omega = 0

    def write_logs(self, timer, new_detection, est_angle, est_dist, angle, dist):
        self.log_file.write("#" * 100 + "\n")
        self.log_file.write(f"Timer             : {timer}\n")
        self.log_file.write(f"Startup status    : {self.startup_done}\n")
        self.log_file.write(f"Last rejected dist: {self.last_rejected_distance}\n")
        self.log_file.write(f"New detection flag: {new_detection}\n")
        self.log_file.write(f"Raw angle         : {angle}\n")
        self.log_file.write(f"Est angle         : {est_angle}\n")
        self.log_file.write(f"Omega             : {self.omega}\n")
        self.log_file.write(f"Raw distance      : {dist}\n")
        self.log_file.write(f"Est distance      : {est_dist}\n")
        self.log_file.write(f"Vx                : {self.vx}\n")


    def mqtt_callback(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            Cx = float(data.get("Cx"))
            distance = float(data.get("distance"))
            angle = np.deg2rad((Cx - X/2) / X * H_FOV) # angle calculation
            #distance = distance * np.cos(angle) # orthogonal projection calculation (assumption)

            with self.lock:
                if not self.startup_done:
                    self.input_filter.add(distance)
                    if self.input_filter.ready():
                        self.startup_done = True
                        self.filter = FilterSetup(self.filter_choice, angle, distance)
                        self.controller = ControlSetup(self.controller_choice,
                                                       self.sp_angle, self.sp_distance)

                if self.startup_done:
                    if self.input_filter.verify(distance):
                        self.raw_angle = angle
                        self.raw_dist = distance
                        self.new_detection = True
                    else:
                        self.last_rejected_distance = distance

            self.last_detection_time = time.time()
            self.detector_active = True
        except:
            pass

    def format_out_data(self, vx, vy, omega):
        return {"vx":vx, "vy":vy, "omega":omega}

    def loop(self):
        timer = time.time()

        while True:
            try:
                if time.time() - timer >= DT:
                    dt = time.time() - timer
                    timer = time.time()

                    if not self.startup_done:
                        continue

                    if time.time() - self.last_detection_time > DETECTOR_TIMEOUT:
                        self.detector_active = False
                        if self.logger_active:
                            self.log_file.write("#" * 100 + "\n")
                            self.log_file.write("Detector Inactive.\n")

                    with self.lock:
                        angle = self.raw_angle
                        dist = self.raw_dist
                        new = self.new_detection
                        self.new_detection = False

                    if self.detector_active:
                        est_angle, est_dist = self.filter.step(angle, dist, new, self.vx, self.vy, self.omega)
                        self.vx, self.vy, self.omega = self.controller.compute(est_dist, est_angle, dt)
                    else:
                        self.vx, self.vy, self.omega = 0, 0, 0

                    self.mqtt.publish(self.format_out_data(self.vx, self.vy, 0))

                    if self.logger_active and self.detector_active:
                        self.write_logs(timer, new, est_angle, est_dist, angle, dist)

            except KeyboardInterrupt:
                self.mqtt.publish(self.format_out_data(0, 0, 0))
                self.log_file.write("#" * 100 + "\n")
                self.log_file.write("Program finished.\n")
                self.log_file.write("#" * 100 + "\n\n\n")
                self.log_file.close()
                break


if __name__ == "__main__":
    SP_ANGLE = 0
    SP_DISTANCE = 0.5
    robot = Follower(SP_ANGLE, SP_DISTANCE, REG_CHOICE, FILTER_CHOICE, LOGGER_ON_OFF)
    robot.loop()
