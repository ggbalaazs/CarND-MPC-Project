# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

### Model Predictive Control Project

The goal of this project is to drive the vehicle around the track. The simulator provides reference points of track center, a third-order polynomial will be fitted to these points resulting in a reference path. The simulator also provides various properties of the car, namely position x/y, orientation and speed. Steering angle and throttle/brake are the two actuators, their current values are known and can be set within limits. A kinematic car model will be built. Then give the state variables, the kinematic equations, the cost function and the reference path an optimizer will be used to find actuator inputs that minimize the cost resulting in an MPC trajectory. The first set of actuator inputs will be executed and this process will be repeated with updated state again-and-again hopefully driving the car mostly in the track center. For realistic purposes 100 msec delay will be used as actuator latency.

### Model

#### State variables
The model contains the following state variables:
* x (car position x)
* y (car postiion y)
* psi (orientation)
* v (speed)
* cte (cross-track error)
* epsi (orientation error)

After transforming the waypoints a car-based orientation system is used with x axis in driving direction and y axis in sideways. This means that `x`, `y` and `psi` is practically zero. Reference polynomial and its derivative evaluated at x = 0 gives `cte` and `epsi` respectively.

#### Actuator inputs

The two actuator inputs are :
* delta (steering), if it is positive in the simulator this implies a right turn
* a (acceleration), being positive/negative, throttle and braking are both handled here

#### Equations

The following kinematic equations are used to update the state variables:
```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] - v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] - v[t] / Lf * delta[t] * dt
```

The optimizer needs initial state variables, constraints and a cost function. For each MPC control loop calculation the initial state variables are the actual state variables. For each timestep in the prediction horizon the constraints must be kept zero and the costs are accumulated.

#### Constraints

The constraints are reformation of the above equations. Each equation can be reordered to a constraint which has zero lower and upper bounds. Let`s take position x for example: 
```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
x[t+1] - (x[t] + v[t] * cos(psi[t]) * dt) = 0
```

So the constraints will be: 
```
x[t+1] - (x[t] + v[t] * cos(psi[t]) * dt) = 0
y[t+1] - (y[t] + v[t] * sin(psi[t]) * dt) = 0
psi[t+1] - (psi[t] - v[t] / Lf * delta[t] * dt) = 0
v[t+1] - (v[t] + a[t] * dt) = 0
cte[t+1] - (f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt) = 0
epsi[t+1] - (psi[t] - psides[t] - v[t] / Lf * delta[t] * dt) = 0
```

#### Cost function

The cost function consists of multiple elements:
* cross-track error and orientation error must be penalized
* car must not halt midway, but must keep a reference speed for continual driving
* minimize the magnitude and change rate of actuators for smooth car motion

As it turns out (beside minimizing `cte`and `epsi`) penalizing large and sudden steering actuation is really important.

```c++
    // minimize cross-track, car heading and velocity error
    for (uint t = 0; t < N; t++) {
      fg[0] += 500 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 1000 * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // minimize the use of actuators
    for (uint t = 0; t < N - 1; t++) {
      fg[0] += 1000 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    // minimize the value gap between sequential actuations
    for (uint t = 0; t < N - 2; t++) {
      fg[0] += 20000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```

###  MPC Trajectory

The prediction horizon is the duration `T` over which future predictions are made. There are two parameters here that need to be tuned, `N` (number of timesteps) and `dt` (time delta between timesteps) as the duration is their product. `T`should be as large as computationally possible (at least a few seconds in this case). `dt` should be small to allow more frequent actuations.

| ref_v (mph) | N | dt (sec) | result
|:--:|:--:|:--:|:--:|
| 40 | 10 | 0.1  | prediction horizon is too short, turns are not handled well |  
| 40 | 20 | 0.1  | too much timesteps, higher optimizer CPU cost adds noticable latency which is not handled at this time |  
| 40 | 10 | 0.5  | prediction horizon is long enough to handle turns, but actuations are rare, car drives very much inner arc |  
| 40 | 10 | 0.25  | prediction horizon is long enough to handle turns, finer actuations, car drives more to the track center |  
| 60 | 10 | 0.25  | higher speed needs more frequent actuations, car drives on inner arc in turns a bit too much |  
| 60 | 10 | 0.2  | ok, good prediction horizon, car drives at track center |  

### Waypoints

The waypoints are transformed from global coords to car-based coords before polynomial fitting is performed.

```c++
          for (uint i = 0; i < ptsx.size(); ++i) {
            x_waypoints[i] = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
            y_waypoints[i] = -(ptsx[i] - px) * sin(psi) + (ptsy[i] - py) * cos(psi);
          }
```

### Latency

For realistic purposes 100 msec delay is set as actuator latency. Without compensation the car heading is going to oscillate more-and-more during the first turn and the car goes off the track. To handle this situation I started to make corrections on the initial state variables:
* `x`, I added `v * dt` to car position x, this is definitely a necessary compensation, but it is not enough in itself, the car did not even make it to the first turn
* `y`, there is no correction needed here because of it is in car-based coords
* `psi`, this is where the magic happens, a correction of `-v / Lf * act_steering * dt` according to the kinematic equation solves the oscillation


