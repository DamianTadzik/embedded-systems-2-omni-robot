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
