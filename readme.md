Hello this is repo for laboratories

Embedded Systems 2 na upelu - ES2_2025



## Project Overview
The project is divided into several software layers, each with a well-defined interface.  
Main goals:
- Controlling the robot via a wireless controller.
- Training a neural network to detect specific objects (in this project: a can of Monster).
- Creating a program that makes the robot drive to a certain object and keep a desired distance.
- Designing and 3D printing accessories (camera station, battery station).

---

## Software Architecture

### L1 — Motion Control Layer
The main task of this layer is controlling the robot via signals directed to the wheels.  
- Receives the control vector **[Vx Vy ω]**, where:
  - Vx, Vy are linear velocities in m/s
  - ω is the angular velocity in rad/s
- This layer runs in a loop. If there is no message on the MQTT topic, the motors are stopped.
- Listens to MQTT topic for control commands.

---

### L2 — Control and Integration Layer
Layer 2 connects Layer 1 and Layer 3.  
- Implements the PID controller to control the robot’s speed and angle of its path.
- Maintains the robot at the desired distance from the object.
- Passes processed control signals to L1.

---

### L3 — Perception Layer
This layer implements the neural network (YOLO) to detect cans.  
- The Hisense camera used in the project provides depth calculation, which is passed to L2.
- The dataset consists of both self-collected and external images:
  - 30 self-collected and annotated images.
  - Additional images sourced from the internet and re-annotated to match labeling standards.

Dataset link: [Monster Net on Roboflow](https://universe.roboflow.com/monster-nzhgs/monster_net-e7vd2)

---

## System Flow
1. Neural network detects the can and outputs position and distance (L3)  
2. PID controller processes this data (L2)  
3. Control vector [Vx Vy ω] is sent to the robot motors (L1)  
4. Robot moves toward the object and maintains distance

---

## Hardware and Accessories
- 3D printed camera station
- 3D printed battery station
- Designed to improve stability and functionality of the robot

---

## Task Allocation

| Name               | Responsibility                           | Layer         |
|---------------------|-------------------------------------------|---------------|
| Damian Brzana       | L1, remote controller, system architecture | L1            |
| Oskar Brandys       | L2                                        | L2            |
| Janusz Pawlicki     | accessories, neural network               | L3            |
| Adrian Paś          | accessories, L2                           | L2            |
| Stanisław Olech     | neural network                            | L3            |

---
