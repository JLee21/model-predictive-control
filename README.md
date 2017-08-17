# Model-Predictive-Control

Student describes their model in detail. This includes the state, actuators and update equations.
## The Overall Model of the Vehicle

The vehicle is run within a 3D Unity program that resembles life-like driving behaviour.
The *state* of the Model consists of the following:

State Variables | Description
-------------- | -----------
x | the x position of the car in global map coordinates measured in meters
y | the y position of the car in global map coordinates measured in meters
psi | the orientation of the car in relation to the global map coordinate
v | the current speed of the car measured in miles per hour

Actuator Variables | Descriptions
-------------- | -----------
accel | the value of the accelerator pedal. In this case, think of `1` being full throttle and `-1` being full brake
delta | the steering angle value in radians that is sent as the steering command

### Update Equations

Hence the `Predictive` in Model-Predictive-Control, the model's state is updated using the following kinematic equations.

![update-eqn]()

Taking into account everything mentioned above, the model's state can be predicted several time steps into the future.
But how can we decide what Accelerator commands (`accel` value) and Steering commands (`delta` value) to send to the model?
Both of these commands affect how the model behaves.

Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

## Tuning the Prediction Amount

The model's predicted state becomes less accurate the further it is predicted into the future.
Also, the model's predicted state is only calculated at discrete points in time. The variables `N` and `dt` address how far the model is predicited into the future. `dt` is the length of time in between each calculated state. `N` is the number of discrete points in time to calculate the model's state. Now the total time into the future is known as `N * dt`. The final chosen `dt` was empirically found to be 200 milliseconds. A value too low would result in unnoticble improvement at the cost of extra computation. With a value too high, the model would often appear sluggish in its reaction to turns causing the model to overreact/overcompensate with a stark steering command.
Similarirly to the `N` value, a high value showed no added handling performance as the accuracy of the model's predicted state dimensioned further in time. Too low of a value may not allow to 'see' several timesteps into the next sharp turn. A value of `N` is 10.

If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.
