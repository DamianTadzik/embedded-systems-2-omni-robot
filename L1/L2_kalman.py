import time
import json
import numpy as np
import paho.mqtt.client as mqtt

# ================= CONFIG =================

MQTT_BROKER = "localhost"
SUB_TOPIC = "robot/nn_output"
PUB_TOPIC = "robot/cmd_vel"

DT = 0.02          # 50 Hz
TIMEOUT = 2.0      # seconds witout detections = stop robot

# Camera horizontal parameters
X_RES = 640
H_FOV = np.deg2rad(87)

# Control parameters
PX_REF = 0.5            # desired distance to the target in meters
KP_X, KD_X = 0.8, 0.3   # control gains for x (forward/backward)
KP_Y, KD_Y = 1.0, 0.3   # control gains for y (left/right)
KP_W = 0.0 #1.2              # control gain for angular velocity

VX_MAX, VY_MAX, W_MAX = 0.4, 0.4, 1.5

# ================= HELPERS =================

def clamp(x, m):
    return max(-m, min(m, x))

def cx_to_angle(cx):
    return (cx - X_RES/2) / X_RES * H_FOV

# ================= KALMAN =================

class Kalman:
    def __init__(self, px, py, std_px=0.10, std_py=0.10, sigma_a=1.5):
        self.x = np.array([px, py, 0.0, 0.0])
        self.P = np.diag([0.5, 0.5, 1.0, 1.0])

        self.std_px = std_px
        self.std_py = std_py
        self.sigma_a = sigma_a

    def predict(self, dt):
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1]
        ])

        sa2 = self.sigma_a**2
        dt2, dt3, dt4 = dt*dt, dt*dt*dt, dt*dt*dt*dt

        Q = sa2 * np.array([
            [dt4/4, 0,     dt3/2, 0    ],
            [0,     dt4/4, 0,     dt3/2],
            [dt3/2, 0,     dt2,   0    ],
            [0,     dt3/2, 0,     dt2  ]
        ])

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z):
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        R = np.diag([self.std_px**2, self.std_py**2])

        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

# ================= L2 =================
import socket
import msgpack
class L2:
    def __init__(self):
        # TELEMETRY UDP SOCKET
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_addr = ("255.255.255.255", 9870)

        # KALMAN FILTER AND STATE
        self.kf = None
        self.last_det = 0
        self.last_cx = None
        self.last_distance_m = None
        self.last_angle_rad = None
        self.last_px = None
        self.last_py = None
        self.meas = None

        # MQTT CLIENT
        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt.on_message = self.on_msg
        self.mqtt.connect(MQTT_BROKER, 1883, 60)
        self.mqtt.subscribe(SUB_TOPIC)
        self.mqtt.loop_start()
    
    def send_telemetry(self, packet):
        try:
            bin_packet = msgpack.packb(packet, use_bin_type=True)
            self.sock.sendto(bin_packet, self.udp_addr)
        except Exception:
            pass        

    def on_msg(self, c, u, msg):
        data = json.loads(msg.payload.decode())
        cx = float(data["Cx"])
        d  = float(data["distance"])

        ang = cx_to_angle(cx)
        px = d * np.cos(ang)
        py = d * np.sin(ang)

        if self.kf is None:
            self.kf = Kalman(px, py)

        self.meas = np.array([px, py])
        self.last_det = time.time()

        # store for telemetry
        self.last_cx = cx
        self.last_distance_m = d
        self.last_angle_rad = ang
        self.last_px = px
        self.last_py = py

    def run(self):
        t = time.time()

        while True:
            now = time.time()
            if now - t < DT:
                time.sleep(DT - (now - t))
                continue
            dt = now - t
            t = now

            if self.kf is None:
                continue

            if now - self.last_det > TIMEOUT:
                self.publish(0, 0, 0)
                self.meas = None
                continue

            self.kf.predict(dt)

            if self.meas is not None:
                self.kf.update(self.meas)
                self.meas = None

            px, py, vpx, vpy = self.kf.x

            vx = KP_X * (px - PX_REF) + KD_X * vpx
            vy = KP_Y * py + KD_Y * vpy
            w  = KP_W * np.arctan2(py, px)

            self.publish(
                clamp(vx, VX_MAX),
                clamp(vy, VY_MAX),
                clamp(w,  W_MAX)
            )

            packet = {
                "timestamp": now,
                "vision": {
                    "cx_px": self.last_cx,
                    "depth_m": self.last_distance_m
                },
                "derived": {
                    "angle_rad": self.last_angle_rad,
                    "px_m": self.last_px,
                    "py_m": self.last_py
                },
                "kalman": {
                    "px": px,
                    "py": py,
                    "vpx": vpx,
                    "vpy": vpy,
                    "P": {
                        "px": float(self.kf.P[0,0]),
                        "py": float(self.kf.P[1,1]),
                        "vpx": float(self.kf.P[2,2]),
                        "vpy": float(self.kf.P[3,3])
                    }
                },
                "control": {
                    "vx": vx,
                    "vy": vy,
                    "omega": w
                },
                "status": {
                    "vision_alive": (now - self.last_det) < TIMEOUT,
                    "dt": dt
                }
            }
            self.send_telemetry(packet)

    def publish(self, vx, vy, w):
        self.mqtt.publish(PUB_TOPIC, json.dumps({
            "vx": vx,
            "vy": vy,
            "omega": w
        }))

# ================= MAIN =================

if __name__ == "__main__":
    L2().run()
