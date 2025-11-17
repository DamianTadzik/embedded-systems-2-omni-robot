# LL – Low Level Motor Control Layer

The LL layer handles the lowest-level control of the robot’s drivetrain. It runs on an Arduino-based board and provides:

- Quadrature encoder counting  
- Motor actuation (PWM + direction)  
- PI-based wheel speed regulation  
- UART command handling from the Jetson computer  
- Telemetry publishing over UART

This module exposes a simple UART protocol for speed commands and maintains real-time regulation of all four wheels.

---

## Features

### 1. Motor Control
Each wheel (TL, TR, BL, BR) has:
- `Direction` pin  
- `PWM` pin  
- Quadrature encoder `A/B` pins

Motor actuation uses signed duty cycles in `[-255, 255]`:

```text
Sign  -> direction
Magnitude -> PWM value (0..255)
```

Apply sign-to-direction and abs(value) to PWM driver.

### 2. Encoder Measurement
- Each encoder uses an interrupt on channel A.
- Speed is computed every `100 ms`.

Computation (example):

```text
speed [RPM] = (Δpulses) * (60 / pulses_per_rev) / dt
pulses_per_rev = 240
dt = measurement interval in seconds (e.g., 0.1 s)
```

### 3. UART Command Protocol
Frame format (bytes):

```text
255  mult  TL  TR  BL  BR
```

- `255` — start byte  
- `mult` — speed scaling factor  
- Each wheel byte in range `[0–200]`, decoded as:

```text
decoded_speed = (value - 100) * mult
```

Notes:
- If no command arrives for > `1 s` → motors stop (failsafe).
- Send the full frame at least every `1 s` to prevent timeout.

### 4. Speed Regulation
- Regulation runs every `100 ms`.
- Modes: 3 modes exist; default is PI regulator.

Controller formula:

```text
u = Kp * e + Ki * ∫e dt
```

- Anti-windup clamping is applied to integral term.
- Controller output `u` is converted to signed duty cycles and applied to motors (clipped to `[-255, 255]`).

### 5. Telemetry
Sent every `1000 ms`.

Telemetry frame (example):

```text
ENC, TL, TR, BL, BR, RPM, TL, TR, BL, BR
```

Contains:
- Raw encoder counts  
- Computed wheel speeds [RPM] for each wheel

---

## File Responsibilities
The LL firmware is responsible for:
- Safe, real-time execution of speed regulation  
- Reliable UART decoding and failsafe handling  
- Motor driver output (direction + PWM)  
- Interrupt-driven encoder counting  
- Telemetry for higher layers (L1–L3)

Important: LL does not make high-level decisions. It executes low-level motion setpoints from upper layers.

---

## Timing

| Task                     | Interval |
|--------------------------|----------|
| Speed regulation         | 100 ms   |
| Telemetry                | 1000 ms  |
| UART timeout → stop      | 1000 ms  |

---

## Integration Notes (Jetson → LL)
- Send the full UART frame every `< 1 s` to prevent failsafe stop.  
- Send values in the format described above.  
- LL guarantees deterministic behaviour and a clean abstraction for higher-level locomotion control.
- Ensure `mult` scaling and wheel byte ranges are respected on the sender side.
