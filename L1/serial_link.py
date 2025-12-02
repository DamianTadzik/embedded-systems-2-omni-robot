import serial

START = 0xAA

def open_serial(port="/dev/ttyACM0", baud=230400):
    return serial.Serial(port, baud, timeout=0.05)

def send_wheel_command(ser, cmds):
    # cmds = [(pwm, dir), ...]
    signed = []
    for speed, direction in cmds:
        percent = (speed / 255.0) * 100.0
        if direction == 0:
            percent *= -1
        signed.append(int(percent))

    frame = bytearray(6)
    frame[0] = START
    frame[1] = signed[0] & 0xFF
    frame[2] = signed[1] & 0xFF
    frame[3] = signed[2] & 0xFF
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
latest_encoders = {
        "tl": 0.0, 
        "tr": 0.0,
        "bl": 0.0,
        "br": 0.0
        }
def serial_reader_task(ser):
    global latest_encoders
    while True:
        enc = read_encoders(ser)
        if enc:    
            latest_encoders["tl"] = enc[0]
            latest_encoders["tr"] = enc[1]
            latest_encoders["bl"] = enc[2]
            latest_encoders["br"] = enc[3]

def start_serial_reader(ser):
    th = threading.Thread(target=serial_reader_task, args=(ser,), daemon=True)
    th.start()
   
