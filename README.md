
# Model Predictive Control (MPC) Project
Using Model Predictive Control (MPC), this project implements a C++ program that can drive a simulated car around a virtual track using specific waypoints from the track itself. The car's actuators have a 100ms latency (delay) that must be accounted for, as well as part of the MPC calculation.

## The Model
The kinematic model includes the vehicle's x and y coordinates, orientation angle (psi), and velocity, as well as the cross-track error and psi error (epsi). Actuator outputs are acceleration and delta (steering angle). The model combines the state and actuations from the previous timestep to calculate the state for the current timestep.
Project Steps:
* Fitting a 3dr order polynomial line, based on road waypoints and evaluating the current state based on that polynomial line.
* Implementing the MPC calculation, including setting variables and constraints.
* Calculating actuator values from the MPC calculations based on current state.
* Accounting for latency (predict state 100ms in the future to replace the current state in the calculation).
* Calculating steering angle & throttle/brake based on the actuator values
* Setting timestep length and duration.
* Testing/tuning of above implementations on Udacity simulator.

## Timestep Length and Elapsed Duration (N & dt)
The values chosen for N and dt are 10 and 0.1, respectively. That was a suggestion of Udacity's office hours for the project. These values mean that the optimizer is considering a one-second duration in which to determine a corrective trajectory. "T" which is N*dt should be a few seconds, at most. Beyond that horizon, the environment will change enough that it won't make sense to predict any further into the future. Given that the car can reach speeds of nearly 100 mph without any extremely erratic driving, this looks to be an excellent spot for N and dt.

## Polynomial Fitting and MPC Preprocessing    
Calculations were simplified by transforming the points from the simulator's global coordinates into the vehicle's coordinates. First, each of the waypoints are adjusted by subtracting out px and py accordingly such that they are based on the vehicle's position. Next, the waypoint coordinates are changed using 2d vector transformation equations:
* ptsx_car[i] = x * cos(-psi) - y * sin(-psi)
* ptsy_car[i] = x * sin(-psi) + y * cos(-psi)

Using the `polyfit()` function, a 3rd order polynomial line is fit to these transformed waypoints, essentially drawing the path the vehicle should try to travel. The cross-track error can then be calculated by evaluating the polynomial function (`polyeval()`) at px, which in this case is now zero. Next, the psi error, or epsi, which is calculated from the derivative of polynomial fit line.

## Model Predictive Control with Latency
For latency, the model accounts for 100ms (0.1 seconds) latency between the actuator calculation and the simulator actually performing that action. To implement this, there is a step to predict where the vehicle would be after 100ms. We set "dt" equal 100ms, then uses the same update equations as those used in the actual MPC model to predicted the state, and fed it back into the true model. Note that these equations were able to be simplified again because of the coordinate system transformation - using x, y and psi all of zero, which makes the equations simpler. This new predicted state, along with the coefficients, are then fed into the `mpc.Solve()` function found in `MPC.cpp`.

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


