import time
from datetime import datetime

# === Import kinematyki ===
from kinematics import omni4_inverse_kinematics

# === MQTT client do odbierania prędkości vx vy omega ===
from mqtt_client import start_mqtt, state
MQTT_BROKER = "localhost"   # lub IP, jeśli to inny komputer
MQTT_PORT = 1883            # klasyczny port MQTT
client = start_mqtt(broker=MQTT_BROKER, port=MQTT_PORT)

# === Serial link do mikrokontrolera ===
from serial_link import open_serial, send_wheel_command, start_serial_reader, latest_encoders
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 230400
serial = open_serial(port=SERIAL_PORT, baud=SERIAL_BAUD)
start_serial_reader(serial)

# === Główna pętla sterująca ===
v_max = 0.5
omega_max = 25.0
dt = 1.0 / 10

time.sleep(2)
while True:
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    vx = state["vx"]
    vy = state["vy"]
    omega = state["omega"]
    print(f"[{ts}] {vx=:.2f} m/s, {vy=:.2f} m/s, {omega=:.2f} rad/s")
    
    tl = latest_encoders["tl"]
    tr = latest_encoders["tr"]
    bl = latest_encoders["bl"]
    br = latest_encoders["br"]
    print(f"\t[ENC] {tl=} {tr=} {bl=} {br=}")

    wheel_cmds, wheel_omegas = omni4_inverse_kinematics(vx, vy, omega, v_max, omega_max)

    send_wheel_command(serial, wheel_cmds)


    time.sleep(dt)
