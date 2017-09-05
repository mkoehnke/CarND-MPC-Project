# CarND Model Predictive Control

This project implements a Model Predictive Control in C++ to maneuver a vehicle around a track using the Udacity Term 2 Simulator, which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

### Observations

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
