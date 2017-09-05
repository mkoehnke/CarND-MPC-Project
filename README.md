# CarND Model Predictive Control

This project implements a Model Predictive Control in C++ to maneuver a vehicle around a track using the Udacity Term 2 Simulator, which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

## Model

The kinematic model that is used in this project includes the x/y coordinates, orientation angle, velocity, crosstrack error and orientation error. The actuators are the steering angle and acceleration. Thus, the state vector has a length of 6.

### Polynomal Fitting

In order to simplify the polynomal fitting, the waypoints received from the simulator had to be transformed to the vehicle's orientation (main.cpp 101-107).

### Latency

Handling the 100 millisecond latency was done by calculating a prediction of a future state vector based on the current state, dt=0.1 and the kinematic model (main.cpp 118-125).

## Parameters

The *N* and *dt* values were choosen manually by trial and error. I started with N=9 (suggestion from the video) and dt=0.5 but found that too error-prone and ended up with: 

N *(timestep length)*| dt *(elapsed duration between timesteps)* 
:---: | :---:
10 | 0.1

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
