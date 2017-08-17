# Model-Predictive-Control

Student describes their model in detail. This includes the state, actuators and update equations.
## The Overall Model of the Vehicle

The vehicle is run within a 3D Unity program that resembles life-like driving behaviour.
The *state* of the Model consists of the following:

state variable | description
-------------- | -----------
x | the x position of the car in global map coordinates measured in meters
y | the y position of the car in global map coordinates measured in meters
psi | the orientation of the car in relation to the global map coordinate
v | the current speed of the car measured in miles per hour

actuator variables | descriptions
-------------- | -----------
accel | the value of the accelerator pedal. In this case, think of `1` being full throttle and `-1` being full brake
delta | the steering angle value in radians that is sent as the steering command



Student describes their model in detail. This includes the state, actuators and update equations.

If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.
