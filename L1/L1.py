import numpy as np
import time
import json
import paho.mqtt.client as mqtt

import time
from datetime import datetime

def omni4_inverse_kinematics(vx, vy, omega, v_max, omega_max):
    """
    Compute wheel speeds and directions for 4-wheel omni (mecanum) robot.

    vx, vy  : linear velocities [m/s]
    omega   : angular velocity [rad/s]
    v_max   : maximum linear velocity [m/s]
    omega_max : maximum wheel angular velocity [rad/s]
    """

    # Stałe geometryczne robota
    wheel_diameter_m = 0.08       # średnica koła
    wheel_base_width_m = 0.24     # szerokość robota
    wheel_base_length_m = 0.14    # długość robota

    # Parametry pomocnicze
    r = wheel_diameter_m / 2      # promień koła
    L = wheel_base_length_m / 2   # pół długości robota
    W = wheel_base_width_m / 2    # pół szerokości robota

    # Macierz kinematyki odwrotnej (dla standardowych kół Mecanum)
    M = np.array([
        [ 1, -1, -(L + W)],
        [ 1,  1,  (L + W)],
        [ 1,  1, -(L + W)],
        [ 1, -1,  (L + W)]
    ])

    # Obliczenie prędkości kątowych kół [rad/s]
    wheel_omegas = (1 / r) * M @ np.array([vx, vy, omega])

    # Normalizacja prędkości, jeśli przekracza maksymalne wartości
    max_omega_val = np.max(np.abs(wheel_omegas))
    if max_omega_val > omega_max:
        wheel_omegas = wheel_omegas * (omega_max / max_omega_val)

    # Konwersja na sygnały sterujące (PWM, kierunek)
    wheel_cmds = []
    for w in wheel_omegas:
        direction = 1 if w >= 0 else 0
        pwm = int(np.clip(abs(w) / omega_max * 255, 0, 255))
        wheel_cmds.append((pwm, direction))

    return wheel_cmds, wheel_omegas  # [(pwm1, dir1), (pwm2, dir2), (pwm3, dir3), (pwm4, dir4)]

# === Globalne prędkości (aktualizowane z MQTT) ===
vx = vy = omega = 0.0


# === Konfiguracja ===
MQTT_BROKER = "localhost"   # lub IP, jeśli to inny komputer
MQTT_PORT = 1883            # klasyczny port MQTT
MQTT_TOPIC = "robot/cmd_vel"
UPDATE_RATE_HZ = 5

# === Callbacki ===
def on_connect(client, userdata, flags, rc):
    print("broker connected! Code:", rc)
    client.subscribe(MQTT_TOPIC)
    print(f"subscribed: {MQTT_TOPIC}")

def on_message(client, userdata, msg):
    global vx, vy, omega
    print(f"Recieved {msg.topic}: {msg.payload.decode()}")
    try:
        data = json.loads(msg.payload.decode())
        vx = float(data.get("vx", 0))
        vy = float(data.get("vy", 0))
        omega = float(data.get("omega", 0))
    except Exception as e:
        print("Invalid MQTT data:", e)

# === Inicjalizacja MQTT ===
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_BROKER, MQTT_PORT)
client.loop_start()
# === Pętla główna 100 Hz ===
v_max = 0.5
omega_max = 25.0
dt = 1.0 / UPDATE_RATE_HZ


import serial
import random
import time
PORT = "/dev/ttyACM0"
BAUD = 9600
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # Poczekaj na reset Arduino

# OLD CONTROL ACTION FOR ORIGINAL FIRMWARE
# def control_action(wheel_cmds):

#     #unpack wheel_cmds into speeds and directions
#     speeds = [random.randint(0,255) for _ in range(4)]  # prędkości TL, TR, BL, BR
#     directions = [random.randint(0, 1) for _ in range(4)]  # kierunki 0 (LOW) lub 1 (HIGH)
#     distance = random.randint(0, 0)  # dystans (choć w 240 nie jest używany)

#     speeds[0] = wheel_cmds[0][0]

#     # Składanie komendy (komenda 240 + 9 wartości)
#     command = [240, distance] + speeds + directions

#     ser.write(bytearray(command))
#     print("Wysłano komendę:", command)

# NEW CONTROL ACTION FOR MY FIRMWARE
def control_action(wheel_cmds):
    """
    wheel_cmds = [TL, TR, BL, BR] in range [-100, 100]
    """
    # multiplier = 1 (normal), -1 (reverse) – możesz wykorzystać np. do odwrócenia sterowania globalnie
    multiplier = 1

    # Zamień (speed, direction) → signed speed w [-100, 100]
    speeds_signed = []
    for speed, direction in wheel_cmds:
        # Przelicz np. 0–255 na -100…100 z kierunkiem
        percent = (speed / 255.0) * 100.0
        if direction == 0:
            percent *= -1
        speeds_signed.append(percent)

    # speeds_signed[1] = -1*speeds_signed[1]
    # speeds_signed[3] = -1*speeds_signed[3]

    # Teraz przeskaluj [-100,100] → [0,200]
    encoded_speeds = [int(percent + 100) for percent in speeds_signed]

    # Ogranicz do zakresu bajtu
    encoded_speeds = [max(0, min(200, s)) for s in encoded_speeds]

    # Ramka: 255 (start), multiplier, TL, TR, BL, BR
    command = [255, multiplier] + encoded_speeds

    ser.write(bytearray(command))
    print("Wysłano komendę:", command)


try:
    while True:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[{ts}]:")
        print(f"    vx = {vx:.3f}  vy = {vy:.3f}  omega = {omega:.3f}")

        wheel_cmds, wheel_omegas = omni4_inverse_kinematics(vx, vy, omega, v_max, omega_max)
        print(f"    wheel_cmds = {wheel_cmds}")

        control_action(wheel_cmds)
        time.sleep(dt)
except KeyboardInterrupt:
    print("Stopping...")
finally:
    client.loop_stop()
    client.disconnect()
    ser.close()
