# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

This repository contains my solution to the MPC project. The vehicle successfully drive a lap around the track. Neither tire leave the drivable portion of the track surface, nor the car pop up onto ledges or roll over any surfaces.

## The Model

The model is the same as described in the course. It includes the state, actuators and update equations as follows:

### The state

The kinematic model includes the vehicle's x and y coordinates, orientation angle (psi), and velocity, as well as the cross-track error and psi error (epsi).

    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];

### The actuators

Actuator outputs are acceleration and delta (steering angle).

### The update equations for the model

    x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v[t+1] = v[t] + a[t] * dt
    cte[t + t] = cte[t] - (y[t] + (v[t] * sin(epsi[t]) * dt));
    epsi[t + 1] = epsi[t] - ((psi[t] - psides[t]) - v[t] * delta[t] / Lf * dt);


### Cost

The cost function based mostly on the vehicle's cross-track error and orientation angle error. It   also included other cost factors suggested in the lessons. Inpiring from PID project, an additional cost penalizing the combination of velocity and delta to improve performance are included.

I used gflags library to optimize parameters without rebuild the whole appliction.


    // cost function
    fg[0] = 0;
    // consider cte, epsi and velocity chenages into cost  keep its value small
    for (int t = 0; t < N; t++) {
      fg[0] += FLAGS_CET_COST_COEFFICIENT * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += FLAGS_EPSI_COST_COEFFICIENT * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += FLAGS_V_COST_COEFFICIENT * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    
    // cosider delta, accelarate into cost to keep its value small.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += FLAGS_DELTA_COST_COEFFICIENT * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += FLAGS_A_COST_COEFFICIENT * CppAD::pow(vars[a_start + t], 2);
      // try adding penalty for speed + steer
      fg[0] += 700 * CppAD::pow(vars[delta_start + t] * vars[v_start + t], 2);
    }
    
    // cosider delta, accelarate changes into cost to make it low oscillatory
    for (int t = 0; t < N - 2; t++) {
      fg[0] += FLAGS_DELTA_DIFF_COST_COEFFICIENT * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += FLAGS_A_DIFF_COST_COEFFICIENT * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }


## Timestep Length and Elapsed Duration (N & dt)

I chosen 10 in timestep length parameter (N) and 0.1s in elapsed duration between timesteps parameter (dt).

The reasoning I the chosen 10 as timestep length is to balance the performance and efficiency of the program. Larger timestep length parameter makes program available to compute more steps. This makes the output trajectory more smooth and accuracy. But it spends more computing resources thus causes the system slowly and more latency. Finally I found that 10 is a good parameter to balance the performance and the efficiency.

To simplify the calculating of the model, I chosen the elapsed duration as the same as the latency of the system, which is 0.1s.

## Polynomial Fitting and MPC Preprocessing

I used Eigen library to solve polynomial fitting to waypoints:


    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
      Eigen::MatrixXd A(xvals.size(), order + 1);
      for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
      }
      for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
        A(j, i + 1) = A(j, i) * xvals(j);
      }
    }
    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
    }

Before I fitting waypoints, I transformed the waypoins from world coordinate to car coordinate to simplify model calculate. After the transforming, the vehicle's x and y coordinates are now at the origin (0, 0) and the orientation angle is also zero.

    Point world_coordinate2vehicle_coordinate(const Point& world_coordinate_point,
        const Point& car_point, const double theta) {
      Point point;
      double dx = world_coordinate_point.x - car_point.x;
      double dy = world_coordinate_point.y - car_point.y;
      point.x = dx * cos(-theta) - dy * sin(-theta);
      point.y = dx * sin(-theta) + dy * cos(-theta);
      return point;
    }

I also use lake_track_waypoint.csv to fit MPC coeffients before I run appliction in simulators. The codes can be see in debug.cpp.


## Model Predictive Control with Latency

The project add a 100 ms latency before the program responsing to the simulator. In other word, the actuations are not changing in this 100 ms. Considerating we use 0.1s as elapsed duration, which is the same as the latency, we can address this latency porblem by using the actuations from the 0.2s, which value is 0.1s in the original kinematic equations.

  AD<double> delta0 = vars[delta_start + t - 1];
  AD<double> a0 = vars[a_start + t - 1];
  if (t > 1) {   // use previous actuations (to account for latency)
    a0 = vars[a_start + t - 2];
    delta0 = vars[delta_start + t - 2];
  }

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

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
