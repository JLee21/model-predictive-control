# Model-Predictive-Control

![gif](https://github.com/JLee21/Model-Predictive-Control/blob/master/img/gif.gif)

## The Overall Model of the Vehicle

The vehicle is run within a 3D Unity program that resembles life-like driving behavior.
The *state* of the Model consists of the following:

State Variables | Description
-------------- | -----------
x | the x position of the car in global map coordinates measured in meters
y | the y position of the car in global map coordinates measured in meters
psi | the orientation of the car in relation to the global map coordinate
v | the current speed of the car measured in miles per hour

Actuator Variables | Descriptions
-------------- | -----------
accel | the value of the acceleration in meters per second squared. In this case, think of `1` being full throttle and `-1` being full brake.
delta | the steering angle value in radians that is sent as the steering command

The above variables are known at every single timestep of the program.

### Update Equations

Hence the `Predictive` in Model-Predictive-Control, the model's state is updated using the following kinematic equations.

![update-eqn](https://github.com/JLee21/Model-Predictive-Control/blob/master/img/update-eqn.JPG)

Taking into account everything mentioned above, the model's state can be predicted several time steps into the future.
But how can we decide what Accelerator commands (`accel` value) and Steering commands (`delta` value) to send to the model?
Both of these commands affect how the model behaves as discuessed below.

## Calculating the Cost of the Model's Prediction

Note only is the model's state known at each time step, but also location markers in the middle of the track known as Way Points. At any given time step, the immediately next 6 Way Points are known. Since the Way Points lie in the middle of the track and evenly spaced, a third-order polynomial can be fitted to these six points. The resulting polynomial, or rather coefficients of the polynomial, represent the reference trajectory for the model as this fitted trajectory aligns closely with the middle of the track's path. Now, not only is the ideal/reference path known but also the predicted path of the model. As discussed earlier, the driving commands (steering angle, accelerator) are optimized to fit the predicted path as closely to the ideal path as possible. Below are the equations used to calculate the cost of the model from the ideal path.

![cost-eqn](https://github.com/JLee21/Model-Predictive-Control/blob/master/img/cost-eqn.JPG)

1) `cte` is the Cross Track Error of the model as defined as the approximate distance from the model's center to the center of the track. `coeffs` is the coefficients of the fitted third-order polynomial and `px` is the x position of the model (here, in relation to the car's reference). The function `polyeval` computes the y value at position `x`. This y values signifies how much lateral distance between the model and the track's center.
2) `depsi` is the desired heading (in radians) of the model. This value is calculated by taking the inverse tangent of the reference trajectory at position `px`. Now, `epsi`, or the error of the model's current heading is calculated as the difference between the desired heading and the current heading.

## How Much To Predict in the Future

The model's predicted state becomes less accurate the further it is predicted into the future.
Also, the model's predicted state is only calculated at discrete points in time. The variables `N` and `dt` address how far into the future the model is predicated. `dt` is the length of time in between each calculated state. `N` is the number of discrete points in time to calculate the model's state. Now the total time into the future is known as `N * dt`. The final chosen `dt` was empirically found to be 200 milliseconds. A value too low would result in unnoticeable improvement at the cost of extra computation. With a value too high, the model would often appear sluggish in its reaction to turns causing the model to overreact/overcompensate with a stark steering command.
Similarly, the `N` value, a high value showed no added handling performance as the accuracy of the model's predicted state dimenished further in time. Too low of a value the model may not 'see' several timesteps into the next sharp turn. A value of `N` is 10 was chosen.

## Order of Operations

To quickly summarize the Order of Operations, the model receives not only the state of itself (x, y positions, etc.) but also the immediately next six Way Points. The Way Points are then used to calculate the reference, or ideal, trajectory of the model (in actuality, the referenced trajectory is the coefficients to a fitted third-order polynomial). Next, the state of the model is predicted one time step into the future in order to compensate for the simulated latency that is common in real driving conditions (discussed in detail later). Now that the model's (latency-incorporated) state and reference trajectory is known the Interior Point OPTimizer (IPOPT) software package solver is able to calculate the best driving commands so as to emulate the reference trajectory.

## Incorporating Simulated Latency

In real driving conditions, when a driving command such as steering is issued by a drive-by-wire system there is inherent lag in time until the command is fully realized by the vehicle's component. In other words, the steering wheel may not actually achieve the commanded steering wheel angle until a fraction of a second into the future. As briefly mentioned earlier, latency compensation is achieved by predicting the model's state into the future by the same amount of latency time (in this case, 100 milliseconds). As a reminder, the model's state is sent to the IPOPT solver and the solver treats this state as the actual current state of the model even though the state is a predicted state 100ms into the future. Below are the equations used to update the model's state to compensate for latency:

```
v *= 0.44704                          (1)
v += accel * LATENCY_SEC              (2)
psi = -v * delta / Lf * LATENCY_SEC   (3)
px = v * LATENCY_SEC                  (4)
```
1) The velocity `v` in miles per hour is first converted to meters per second
2) Here, the predicted velocity is approximated to change only in the forward direction of the car (x-direction). Given a small time step into the future, `LATENCY_SEC`, and the current acceleration of the model, `accel`, the predicted velocity is calculated.
3) `psi` is the bearing of the car in radians. `delta` is the current steering wheel command in radians and Lf is a constant related to the model's turning radius at constant velocity.
4) Similar to the predicted velocity, the model's x position, `px`, is approximated to be entirely in the forward direction (x-direction). In addition, the model's y position is not calculated since the model does not shift laterally to any noticeable degree.
