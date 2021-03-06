# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## PID-Controller for Steering Angle

![](https://media.giphy.com/media/8N7fTPAIHCucw/giphy.gif)

## Reflection

Influence of controller components on the control circuit:
* Proportional: The P part of the controller ensures a response that is proportional to the tracking error. 
However for big tracking errors
the controller can become too big and the controller overreacts. 
In this case the controll circuit can get out of control or get stuck in an oscillating state.

* Integral: The I part is integrating the tracking errors. This ensures that the control circuit reaches it's desired state. 
Otherwise a throttle controller wouldn't reach
 it's desired final velocity if a constant disturbance (e.g. wind resistance) is present. 
 The I part reacts slowly since it has to gain momentum over time.

 * Differential: The D part reacts proportional to the change of the tracking error. Therefore the D part reacts fast to changes. 
 However it wouldn't create a response if the error is constant over time.
 Even if the error consists over time, the D part would not create a control response.

* Parameter choice: The parameters for the P, I and D component were chosen by hand. Starting point was the parameter set from the lesson. 
Then each parameter was manipulated in both directions.

Since the simulator expects steering values in the interval [-1, 1] the control response R is mapped through tanh(R). 
The result is a slightly nonliner response, however the result did not differ noticeably
when compared to a clipping at the interval's borders. 

![](testrun.gif)

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

