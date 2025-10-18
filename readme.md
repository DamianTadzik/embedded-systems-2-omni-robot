Hello this is repo for laboratories

Embedded Systems 2 na upelu - ES2_2025



# PLAN

- Plan is to have few layers of software interconnected with defined interfaces

- L1 is layer that gets [Vx Vy omega] vector that contains velocities in m/s and omega in rad/s which are control signals for the robot. L1 runs in the loop, if there is nothing on the MQTT topic it should stop the motors
_ L1 listens to the MQTT topic


# Dataset

The dataset used to train the YOLO network consists of both self-collected and externally sourced images. A total of 30 images were collected and annotated by us, while the remaining images were gathered from the internet and, in some cases, re-annotated to ensure consistency with our labeling standards.

Dataset link: https://universe.roboflow.com/monster-nzhgs/monster_net-e7vd2