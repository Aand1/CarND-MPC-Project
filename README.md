# CarND-Controls-MPC

This project implements a _Model Predictive Controller_ (MPC) to drive a car over a predefined route. The route is given as a sequence of _waypoints_ `{(x[0], y[0]), ..., (x[k], y[k]), ..., (x[n], y[n])}` in the car's local reference frame, defined as a two-dimensional Cartesian coordinate system with the X axis pointing forward from the car's center and the Y axis pointing to the left. The car's own state is defined as <code>(x, y, &theta;, v)</code>, where <code>(x, y, &theta;)</code> represent its pose in the global reference frame, and `v` is the current linear velocity. See Figure 1 below for an illustration.

<img src="https://xperroni.github.io/CarND-MPC-Project/frames.jpg">

The output of the MPC is a sequence of actuations `{(a[0], d[0]), ..., (a[k], d[k]), ..., (a[m], d[m])}`, where `a[k]` is an acceleration in meters per second squared, and `dk` a turn rate in radians per second. These update the car's state according to the formulas below:

<ol>
<li><code>v[t+1] = v[t] + a[t] * dt</code></li>
<li><code>&theta;[t+1] = &theta;[t] + v[t+1] * Lf * d[t] * dt</code></li>
<li><code>x[t+1] = x[t] + cos(&theta;[t+1]) * v[t+1] * dt</code></li>
<li><code>y[t+1] = y[t] + sin(&theta;[t+1]) * v[t+1] * dt</code></li>
</ol>

Where `dt` is the time difference between `t` and `t+1` in seconds, and `Lf` is the contribution factor of the linear speed to the turning rate.

The MPC parameters `dt` (duration of a time step) and `N` (number of time steps) were set to `0.1` and `15` respectively. Together they specify a prediction time window of `1.5` seconds. The value of `dt` was chosen to coincide with the system latency (see below for details), while `N` was determined empirically as a good balance between effective planning and route stability &ndash; larger values for `N` tend to cause the car to wobble about the road, as the planner constantly changes near-term actuation plans to better fit the overall reference route.

As mentioned earlier, the steering system has a `0.1` second latency between an actuation command being issued and entering into effect. Since the MPC time step length is set to the same value, this delay can be accounted for simply by using the _second_ actuation pair `(a[1], d[1])` to steer the car, instead of the first as would normally be done. This assumes that the acceleration and turning rate values currently in use are precisely `(a[0], d[0])` (i.e. the first actuation pair), which might not always be the case, especially when the car has just started and is still stopped; in practice, however, this approximation was found to be sufficient for our purposes.

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

On BASH-enabled environments you may also run the `build.sh` script to build the project and create a symbolic link to the executable in the base project directory.
