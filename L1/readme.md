# L1 – High-Level Velocity Interface

The L1 layer is responsible for receiving high-level velocity commands (vx, vy, omega) and converting them into low-level wheel commands for the Arduino/LL layer.  
It supports two control sources:

- Web-based controller (app.html via MQTT/WebSockets)
- L2 layer (main system; preferred in real operation)

L1 performs inverse kinematics for a 4-wheel mecanum robot and transmits the resulting wheel commands to the LL layer via UART.

## Project Structure

```
L1/
 ├── L1.py
 ├── app.html
 └── mqttws31.min.js
```

### L1.py

Python application that:

- Subscribes to MQTT topic `robot/cmd_vel`
- Parses JSON `{vx, vy, omega}`
- Computes wheel speeds using inverse kinematics
- Converts wheel speeds → signed speed commands
- Encodes a UART packet:

```
[255, multiplier, TL, TR, BL, BR]
```

- Sends it to LL over `/dev/ttyACM0`

L1 runs at 5 Hz by default.

### app.html

A standalone web UI that:

- Connects to the MQTT broker over WebSockets
- Provides:
    - Virtual joystick (vx, vy)
    - Slider for rotation (omega)
    - Telemetry panel
- Publishes JSON commands:

```
{ "vx": ..., "vy": ..., "omega": ... }
```

to the topic `robot/cmd_vel` every 100 ms.

Used mainly for debugging and teleoperation.

### mqttws31.min.js

Local Paho MQTT JavaScript client used by the web interface.

## How to Run L1

1. Start Mosquitto (MQTT broker)

Make sure the MQTT broker is running (see MQTT README):

```
mosquitto -v -c my_mosquitto.conf
```

2. Run the L1 Python application

```
python3 L1.py
```

Requirements:

```
pip install paho-mqtt pyserial numpy
```

L1 will:

- Connect to MQTT (`localhost:1883`)
- Open `/dev/ttyACM0`
- Start converting and sending wheel commands to LL

## How to Use the Web Controller

Simply open the file:

`app.html`

The page:

- Connects to the broker via WebSockets (`ws://<jetson_ip>:9001`)
- Sends joystick and slider commands to `robot/cmd_vel`
- Displays sent values and connection status

No hosting is required; the file works when opened directly in a browser.

## Data Flow

From Web → L1 → LL → Motors

```
app.html
        ↓ MQTT/WebSockets (9001)
Mosquitto broker
        ↓ MQTT (1883)
L1.py
        ↓ UART
LL (Arduino)
        ↓ PWM signals
Motors
```

From L2 → L1 → LL

The primary use case:

```
L2 (main motion planner)
        ↓ MQTT (cmd_vel)
L1 (kinematics + UART)
        ↓
LL (firmware)
```

The web controller is mainly for testing when L2 is unavailable.

## Inverse Kinematics Model

L1 uses standard mecanum kinematics:

```
[ w1 ]   [  1  -1  -(L+W) ]   [ vx ]
[ w2 ] = [  1   1   (L+W) ] * [ vy ]  / r
[ w3 ]   [  1   1  -(L+W) ]   [ omega ]
[ w4 ]   [  1  -1   (L+W) ]
```

Wheel angular velocities are normalized and mapped to:

- PWM [0…255]
- Direction bit
- Signed speed [-100…100]
- Encoded UART data [0…200]

## UART Command Format (L1 → LL)

```
byte[0] = 255          // Start byte
byte[1] = multiplier   // Normally 1
byte[2] = TL
byte[3] = TR
byte[4] = BL
byte[5] = BR
```

Each wheel speed:

signed_percent [-100..100] → encoded [0..200]

## Summary

The L1 layer is a bridge between high-level velocity control and low-level wheel actuation:

- Accepts vx, vy, omega
- Computes wheel commands
- Encodes and sends them to LL firmware
- Allows both L2 and a web interface to control the robot
- Uses MQTT for messaging and UART for motor command delivery
