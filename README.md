# CarND-Controls-MPC

## Executive Summary

This repository contains my implementation for the MPC Project. In this project a vehicle [simulator](https://github.com/udacity/self-driving-car-sim/releases) navigates around a curved track in autonmous mode, transmitting telemetry and track  data via C++ websockets, constantly sending the steering angle and throttle back to the simulator to control the vehicle. 

### Model

The model is a kinematic bicycle model implemactuation commands. The error minimization is achieved using a 3rd degree polynomial fit for the x and y waypoints. The waypoints are calculated for the specified duration and intervals using the vehicle's x and y coordinates, orientation angle (psi), and velocity, the cross-track error (cte) and psi error (epsi). The model has to take into consideration a latency of 100ms for the actuator outputs to be transmitted to the simulator. State and actuation values from the previous timestep are combined to calculate the state for the current timestep as given in the equations below:

### Equations

```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t]/L_f * delta[t] * dt
v[t+!] = v[t] + a[t] *dt
cte[t+!] = f(x[t]) - y[t] + (v[t] * sin(epsi[t])) * dt
epsi[t+1] = psi[t] - psides[t] + ((v[t]/L_f) * delta[t] * dt)

```
* x, y : position of the car along the axes
* psi - heading direction
* v - velocity
* cte - cross-track error
* epsi - orientation error
* L_f - distance from center of mass of the vehicle to front tires

### Timestep Length and Elapsed Duration (N & dt)

A range of values were tried for N and dt. With N at 20 and dt at 0.2, the vehicle was veering off track. 10/0.1 was reasonable but 10/0.05 had big errors. A lot of intermediate values were tried on but the behavior was very erratic. Finally the model was tuned using 15/0.15 and had a good performance.

### Polynomial Fitting and MPC Preprocessing 

The waypoints are obtained in the frame of the vehicle by transforming them to the vehicle's perspective. This transformation was done by shifting the origin to the current vehicle position and rotating to align the x-axis with heading direction. With the x and y coordinates at the origin fitting a 3rd degree polynomial to the waypoints was simplified. All this code can be found in main.cpp, lines 109-127

### Model Predictive Control with Latency

A couple approaches were used to account for latency - controlling the speed and updating the actuations for timesteps past the first (MPC.cpp Lines 104-107) so that the actuations are applied a timestep later.


---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: Code only compiles with gcc\g++ 6. If you have v7 installed, used this command to explicitly compile with g++6

cmake -DCMAKE_C_COMPILER="/usr/bin/gcc-6" -DCMAKE_CXX_COMPILER="/usr/bin/g++-6" . &&make

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

