# SDCND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Summary

This repository contains a C++ implementation of a model predictive controller to be used with the Udacity simulator available at https://github.com/udacity/self-driving-car-sim. It has been implemented as part of the Udacity Self-Driving Car Engineer Nanodegree Program.

The goal of the project was to implement and configure the MPC in such a way that the car successfully drives a lap around the track. No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). See video on Youtube for a test lap:

[![SDCND - Project 9 - Model Predictive Control ](https://img.youtube.com/vi/1vvB_LUanaU/0.jpg)](https://www.youtube.com/watch?v=1vvB_LUanaU "SDCND - Project 9 - Model Predictive Control ")

## The model

The model used in this project is a non-linear kinematic bicycle model: it does not take into account tire forces, gravity, and mass.

The equations describing this model can be found in MPC.cpp:

```
fg[2 + x_start + i] = x1 - (x0 + v0*CppAD::cos(psi0)*dt);
fg[2 + y_start + i] = y1 - (y0 + v0*CppAD::sin(psi0)*dt);
fg[2 + psi_start + i] = psi1 - (psi0 - v0*delta0 / Lf*dt);
fg[2 + v_start + i] = v1 - (v0 + a0*dt);
fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) - v0*delta0 / Lf*dt);
```

The Lf costant (2.67) was obtained by measuring the radius formed by running the vehicle in the simulator around in a circle with a constant steering angle and velocity on a flat terrain. Lf was tuned until the the radius formed by the simulating the model presented in the classroom matched the previous radius. This is the length from front to CoG that has a similar radius.

## Timestep Length and Elapsed Duration (N & dt)

The N and dt values are used to define the prediction horizon (T = N * dt), in other words how many seconds the MPC should look into the future and predict the vehicle trajectory. A trade-off must be found between vehicle stability and computational load. After some tries I found that the following values allow the car to drive safely up to 110 mph on the test track:

* N = 10
* dt = 1

In order to cope with higher values for N and dt, I tried as well to allow the polynomial fitting library to spend more time ("Numeric max_cpu_time"), but this did not lead to any significant improvement.

## Polynomial Fitting and MPC Preprocessing

In order to simplify the equation, all values are first transformed to the car's reference system (shift + rotation). This makes possible to have x, y, and psi initially set to 0:

```
state << 0, 0, 0, v, cte, epsi;
```

Please note that, when dealing with latency, the array of values defining the state are:

```
state << x_latency, y_latency, psi_latency, v, cte, epsi;
```

## Model Predictive Control with Latency

In order to better simulate a real vehicle, the MPC implementation allows to set a value for the actuators' latency. In order to correctly handle that scenario, I took into account the latency in the following code:

```
int latency = 100; // latency in milliseconds between stimulus and actuation
double latency_in_seconds = (double) latency / 1000; // latency in seconds, to be used in the calculations

double x_latency = 0 + v * cos(0) * latency_in_seconds; // x0 + v * cos(psi0) * dt
double y_latency = 0 + v * sin(0) * latency_in_seconds; // y0 + v * sin(psi0) * dt
double psi_latency = 0 - v * steer_value / Lf * latency_in_seconds; // psi0 - v * steer_value / Lf * dt

v += throttle_value * latency_in_seconds; // predicted v after dt
```

Even though this is a naive solution, it works quite well for latency values in the range 0-200 ms.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
