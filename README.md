# CarND Model Predictive Control

This project implements a Model Predictive Control in C++ to maneuver a vehicle around a track using the Udacity Term 2 Simulator, which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

## Model

The model that is used in this project is a *kinematic model* that is a simplification of a dynamic model which ignore tire forces, gravity, and mass. This simplification reduces the accuracy of the models, but it also makes them more tractable.

The vehicle state vector consists of 4 values:

- **x (px)**: x-coordinate of the vehicle position
- **y (py)**: y-coordinate of the vehicle position
- **psi**: heading / orientation angle of the vehicle
- **v**: velocity of the vehicle

In addition, there are two actuator outputs:

- **delta**: steering angle of the vehicle (*-25* degrees for full left steering and *25* degrees for full right steering)
- **a**: acceleration of the vehicle (*-1* for full brake and *1* for full acceleration)

In order to predict the state for the next time step, the following update equations have been used: 

```
x1 = (x0 + v0 * CppAD::cos(psi0) * dt);
y1 = (y0 + v0 * CppAD::sin(psi0) * dt);
psi1 = (psi0 - v0 * delta0 / Lf * dt);
v1 = (v0 + a0 * dt);
```

They take current state and the actuators into account to predict the next state. Also, `dt` is the elapsed duration between predictions and `Lf` is the distance between the front and center of the gravity of the vehicle, which describes the steering radius. 


To calculate an optimal trajectory for the given waypoints and the vehicle's kinematic model, two errors need to be minimized using a cost function.

Those errors are:

- **cte**: crosstrack error, that is the distance from the vehicle position to the track waypoints
- **epsi**: orientation error of the vehicle

In order to update those errors for the next time step, the following update equations were used:

```
cte1 = ((f0-y0) + (v0 * CppAD::sin(epsi0) * dt));
epsi1 = ((psi0 - psi_des0) - v0 * delta0 / Lf * dt);
```

### Polynomal Fitting

To simplify the polynomal fitting, the waypoints received from the simulator had to be transformed to the vehicle's orientation (main.cpp 101-107).

### Latency

Handling the 100 millisecond latency was done by calculating a prediction of a future state vector based on the current state, dt=0.1 and the kinematic model (main.cpp 118-125).

## Choosing N/dt Parameters

The *N* and *dt* values were choosen manually by trial and error:

N *(timestep length)*| dt *(elapsed duration between timesteps)* 
:---: | :---:
10 | 0.1

Explanation: *N \* dt* is called a prediction horizon. 100ms * 10 means that in order to calculate an optimal trajectory, the optimizer is considering a duration of 1 second.

In general, with a small prediction horizon the controller can calculate the trajectory much quicker, but is less accurate and less stable. Large prediction horizons are much more stable and smooth, but they increase the computational time and latency. 

Before finding the mentioned values, I started with N=9 (suggestion from the video) and dt=0.5 but found that too error-prone. Also other values like N=20 / dt=1, N=15 / dt=0.8 etc. didn't work well, especially with faster speed. 

## Observations

Previous attempts to set *N* to a higher value like *20* resulted in errors in orientation and track reference:

![PID Simulator](https://github.com/mkoehnke/CarND-MPC-Project/raw/master/resources/mpc-N-high.png)

In addition, setting *dt* to a higher value like *0.5* resulted in similar issues:

![PID Simulator](https://github.com/mkoehnke/CarND-MPC-Project/raw/master/resources/mpc-dt-high.png)

In general, the current implementation doesn't behave well when the speed is higher then **30**, especially towards tight turns: 

![PID Simulator](https://github.com/mkoehnke/CarND-MPC-Project/raw/master/resources/mpc-high-speed.png)


## Installation

Install [CppAD](https://www.coin-or.org/CppAD/) and [Ipopt](https://projects.coin-or.org/Ipopt):

```
brew install cppad
brew install ipopt
```

In addition, you have to install the uWebSocketIO using the following command in the root directory of the project:

```
sh install-mac.sh
```

Compile the source code by entering the following commands:

```
mkdir build
cd build
cmake ..
make
./mpc
```

![PID Simulator](https://github.com/mkoehnke/CarND-MPC-Project/raw/master/resources/mpc.png)
