import time
from datetime import datetime

# === Import kinematyki ===
from kinematics import omni4_inverse_kinematics

# === MQTT client do odbierania prędkości vx vy omega ===
from mqtt_client import start_mqtt, mqtt_request
MQTT_BROKER = "localhost"   # lub IP, jeśli to inny komputer
MQTT_PORT = 1883            # klasyczny port MQTT
client = start_mqtt(broker=MQTT_BROKER, port=MQTT_PORT)

# === Serial link do mikrokontrolera ===
from serial_link import open_serial, send_wheel_command, start_serial_reader, encoders_feedback
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 230400
serial = open_serial(port=SERIAL_PORT, baud=SERIAL_BAUD)
start_serial_reader(serial)

# === Główna pętla sterująca ===
dt = 1.0 / 10

time.sleep(2)
while True:
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    vx = mqtt_request["vx"]
    vy = mqtt_request["vy"]
    omega = mqtt_request["omega"]
    print(f"[{ts}] {vx=:.2f} m/s, {vy=:.2f} m/s, {omega=:.2f} rad/s")
    
    tl = encoders_feedback["tl"]
    tr = encoders_feedback["tr"]
    bl = encoders_feedback["bl"]
    br = encoders_feedback["br"]
    print(f"\t[ENC RAW] {tl=} {tr=} {bl=} {br=}")

    wtl = encoders_feedback["wtl"]
    wtr = encoders_feedback["wtr"]
    wbl = encoders_feedback["wbl"]
    wbr = encoders_feedback["wbr"]
    print(f"\t[ENC SPD] {wtl:.2f} rad/s, {wtr:.2f} rad/s, {wbl:.2f} rad/s, {wbr:.2f} rad/s")

    omegas = omni4_inverse_kinematics(vx, vy, omega, max_wheel_omega_rad_per_s=1.0)
    
    rwtl = omegas[0]
    rwtr = omegas[1]
    rwbl = omegas[2]
    rwbr = omegas[3]
    print(f"\t[REQ WH SPD] {rwtl:.2f} rad/s, {rwtr:.2f} rad/s, {rwbl:.2f} rad/s, {rwbr:.2f} rad/s")

    # OPEN LOOP CONTROL - map wheel angular velocities to PWM duty cycles
    # Now map the omegas to PWM commands in range [-127, 127]
    duty_cycles = []
    for w in omegas:
        pwm = int(max(-127, min(127, w * 127)))  # Simple linear mapping
        duty_cycles.append(pwm)

    dctl = duty_cycles[0]
    dctr = duty_cycles[1]
    dcbl = duty_cycles[2]
    dctr = duty_cycles[3]
    print(f"\t[WHL DUTY] {dctl=} {dctr=} {dcbl=} {dctr=}")

    send_wheel_command(serial, duty_cycles)

    time.sleep(dt)
