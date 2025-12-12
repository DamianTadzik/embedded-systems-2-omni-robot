# L1 – High-Level Velocity & Wheel Speed Control

The **L1 layer** is responsible for closing the **wheel speed control loop** on the host computer (Jetson/PC).
It receives high-level velocity commands `(vx, vy, omega)`, converts them into **desired wheel angular speeds**, and **regulates each wheel using PID controllers** based on encoder feedback from the LL (Arduino) layer.

L1 therefore no longer operates in open-loop PWM mode – **wheel speed control is closed in L1**.

---

## Responsibilities of L1

* Receive high-level velocity commands `(vx, vy, omega)` via MQTT
* Compute desired wheel angular velocities using mecanum inverse kinematics
* Read encoder feedback from LL over UART
* Compute wheel angular speeds `[rad/s]`
* Run **4 independent PID controllers** (one per wheel)
* Send **PWM duty cycles** to LL

---

## Control Architecture

```
(vx, vy, omega)
        ↓
   Inverse Kinematics
        ↓
Desired wheel speeds [rad/s]
        ↓
   PID (per wheel)
        ↓
   PWM duty cycles
        ↓
        LL (Arduino)
        ↓
     Motors + Encoders
        ↑
   Encoder feedback (UART)
```

---

## Project Structure

```
L1.py              # Main control loop (IK + PID + UART)
PID.py             # Simple PID controller implementation
kinematics.py      # Mecanum inverse kinematics
serial_link.py     # UART protocol + encoder decoding
mqtt_client.py     # MQTT interface (cmd_vel)
app.html           # Web joystick (debug / teleop)
TODO.md            # Planned improvements
```

---

## MQTT Interface

### Topic

```
robot/cmd_vel
```

### Message format

```json
{ "vx": 0.0, "vy": 0.0, "omega": 0.0 }
```

* `vx`, `vy` – linear velocities [m/s]
* `omega` – angular velocity [rad/s]

Commands may originate from:

* **L2 motion planner** (normal operation)
* **Web UI** (`app.html`) for testing

---

## UART Interface (L1 ↔ LL)

### L1 → LL

* Sends PWM duty cycles for 4 wheels
* Range: `[-127, 127]` (mapped internally to signed percentage)

### LL → L1

* Sends raw encoder counts (16-bit, with wrap-around)
* L1 computes:

  * wheel angular speed `[rad/s]`
  * loop `dt`

All encoder handling and speed estimation is done in **serial_link.py**.

---

## PID Control

* One PID per wheel: `TL`, `TR`, `BL`, `BR`
* Inputs:

  * Target wheel speed `[rad/s]`
  * Measured wheel speed `[rad/s]`
* Output:

  * PWM duty cycle command

Current gains (to be tuned):

```
kp = 1.2
ki = 12.0
kd = 0.04
output_limit = 127
```

---

## Web Controller (Debug Only)

`app.html` provides:

* Virtual joystick → `(vx, vy)`
* Slider → `omega`
* MQTT over WebSockets

Used **only for debugging and teleoperation** when L2 is unavailable.

---

## How to Run

### 1. Start MQTT broker

```
mosquitto -v -c my_mosquitto.conf
```

### 2. Run L1

```
python3 L1.py
```

Dependencies:

```
pip install paho-mqtt pyserial numpy
```

L1 will:

* Connect to MQTT (`localhost:1883`)
* Open `/dev/ttyACM0`
* Start encoder reader thread
* Close the wheel speed control loop

---

## Data Flow Summary

```
L2 / Web UI
      ↓ MQTT
     L1 (IK + PID)
      ↓ UART
     LL (Motor driver)
      ↓
    Motors
```

---

## Notes

* L1 is now a **real-time wheel speed regulator**, not just a kinematics bridge
* LL firmware is simplified (no PID inside Arduino)
* Timing quality depends on host OS scheduling
* Future improvement: move PID to LL if hard real-time is required
