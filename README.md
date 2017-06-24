# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Description

Every timestamp `dt` we get from simulator collections of waypoints position (`ptsx`, `ptsy`) and vehicle position (`px`, `py`), velocity (`v`), direction (`psi`), steering angle (`delta`), acceleration (`a`). To easy calculation we convert waypoints from map coordinates to [car's coordinates](src/main.cpp#L118). All calculations then are done in vehicle coordinates, so in our state position and direction are zeros. Initial `cte` is just waypoints position related to car's position, and we can calculate `epsi` as `atan` of waypoints derivative.

To get waypoints line, we fit it to the third order polynomial. It is good enough to describe all major roads. Speed was converted from mph to mps since all calculations are done with meters. To deal with 100 milliseconds latency initial state was slightly changed. Instead of sending car's state at time frame 0.0 we predict vehicle state in 100 milliseconds according to [kinematic equations](src/main.cpp#L139) and send it as car's current state.

Car's speed is not high, so `dt=0.4` is good to handle changing situation. There is no reason to predict car's position in more than 2 sec, so `N=6`. Cost function [was chosen](src/MPC.cpp#L54) to minimize cte and epsi. To avoid car's stop reference speed has been selected as 25 mps and included in the cost function. We do not want to have high values for steering angle and acceleration, so they are added to the cost function as well. And lastly, to have smooth behavior car should not change steering angle and acceleration drastically, so we added change minimization in the cost function.

Code was modified to tune parameters [without compilation](src/main.cpp#L72). I noticed that `epsi` is the most important parameter to control car's behavior. Eventually, it is the only parameter which was tuned.

## Result

[Video](https://youtu.be/vyoi-Ck825E) of car with referral speed 25mps.

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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.
