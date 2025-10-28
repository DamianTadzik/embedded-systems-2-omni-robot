L2- Second layer

Responsible for controling velocities of the robot's frame of reference based on the distance from detected object from the robot and on location of it's center in the frame.



INPUTS:

&nbsp;	- Distance from the detected object (distance)

&nbsp;	- Location of center of detected object in the frame (C(x,y))

OUTPUTS:

&nbsp;	- vx - translational velocity along x axis of robot's reference frame

&nbsp;	- vy - translational velocity along y axis of robot's reference frame

&nbsp;	- omega - rotational velocity along z axis of robot's reference frame



Script consists of angle and distance regulators (which are implemented, based on choice, with two-state position controllers or PID controllers), which operate on angle and distance variables. Both are calculated with help of inputs and knwoledge of camera's FOV.



