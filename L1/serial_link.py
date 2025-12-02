import serial

START = 0xAA

def open_serial(port="/dev/ttyACM0", baud=230400):
    return serial.Serial(port, baud, timeout=0.05)

def send_wheel_command(ser, cmds):
    """    
    Sends a command frame to the robot with the desired wheel speeds.

    :param ser: Serial object
    :param omegas: cmds is four-element array-like of wheel PWMs in range [-127, 127]
    """    
    signed = []
    for speed in cmds:
        percent = (speed / 127.0) * 100.0
        signed.append(int(percent))

    frame = bytearray(6)
    frame[0] = START
    frame[1] = -signed[0] & 0xFF
    frame[2] = signed[1] & 0xFF
    frame[3] = -signed[2] & 0xFF
    frame[4] = signed[3] & 0xFF
    checksum = (START + frame[1] + frame[2] + frame[3] + frame[4]) & 0xFF
    frame[5] = checksum

    ser.write(frame)

def read_encoders(ser):
    """
    Odbiera ramkę enkoderów:
    0x55, TL_lo, TL_hi, TR_lo, TR_hi, BL_lo, BL_hi, BR_lo, BR_hi, checksum
    Zwraca tuple: (tl, tr, bl, br) lub None przy błędzie.
    """
    if ser.in_waiting < 10:
        return None

    header = ser.read(1)
    if not header or header[0] != START:
        return None

    data = ser.read(9)
    if len(data) != 9:
        return None

    tl = data[0] | (data[1] << 8)
    tr = data[2] | (data[3] << 8)
    bl = data[4] | (data[5] << 8)
    br = data[6] | (data[7] << 8)

    checksum = (START +
                data[0] + data[1] +
                data[2] + data[3] +
                data[4] + data[5] +
                data[6] + data[7]) & 0xFF

    if checksum != data[8]:
        print(f"[WARNING] {checksum=} {data[8]=}")
        return None

    return (tl, tr, bl, br)


import threading
import time
import math
encoders_feedback = {
    # raw encoder counts
    "tl": 0.0, "tr": 0.0,
    "bl": 0.0, "br": 0.0,

    # angular speeds [rad/s]
    "wtl": 0.0, "wtr": 0.0,
    "wbl": 0.0, "wbr": 0.0,

    # loop delta time
    "dt": 0.0    
}
CPR = 230
RAD_PER_COUNT = 2 * math.pi / CPR

def serial_reader_task(ser):
    last_time = time.time()
    last_enc = None

    while True:
        enc = read_encoders(ser)   # returns [tl, tr, bl, br]
        if not enc:
            continue

        # dt
        now = time.time()
        dt = now - last_time
        last_time = now
        encoders_feedback["dt"] = dt

        # raw counts
        tl, tr, bl, br = enc
        encoders_feedback["tl"] = tl
        encoders_feedback["tr"] = tr
        encoders_feedback["bl"] = bl
        encoders_feedback["br"] = br

        # speeds
        if last_enc is not None:
            encoders_feedback["wtl"] = (tl - last_enc[0]) / dt * RAD_PER_COUNT
            encoders_feedback["wtr"] = (tr - last_enc[1]) / dt * RAD_PER_COUNT
            encoders_feedback["wbl"] = (bl - last_enc[2]) / dt * RAD_PER_COUNT
            encoders_feedback["wbr"] = (br - last_enc[3]) / dt * RAD_PER_COUNT
        else:
            encoders_feedback["wtl"] = 0.0
            encoders_feedback["wtr"] = 0.0
            encoders_feedback["wbl"] = 0.0
            encoders_feedback["wbr"] = 0.0

        last_enc = enc

def start_serial_reader(ser):
    th = threading.Thread(target=serial_reader_task, args=(ser,), daemon=True)
    th.start()
   
