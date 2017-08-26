# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Rubric

### PID Controller components


* **P (proportional) component.**
The component commands the car to turn in the direction opposite to the given error. If only P component is set, car will try to stay along given track. But it overshoots easily, so the oscillations appear that can increase significantly with increasing the speed. Check [this video](./videos/no_d.mov) to see how car moves with only P component set.
* **D (differential) component.**
This component allows to smooth oscillations described previously. The higher is the value of the component, the steering angle of more value car tries to apply to reduce the differential error.
* **I (integral) component.**
The component reduces the error of systematic bias that can exist in the system. There is no systematic bias in the simulator, so setting the I component leads to larger error instead. Check [this video](./videos/i_added.mov) to see how car runs with I value set to 1.0.

### Choosing the hyperparameters

For choosing P, I, D components values the twiddle technique was used. Car was run in the simulator for 10000 'steps' and the resulted error was evaluated. Then the application restarted the simulator remotely and the next run with modified parameters was performed. It was repeated until time runs out or the sum of parameters difference values met the threshold. The twiddle code is located in `main.cpp` and can be run if `isSteerTwiddle` set to `true`.

However, during tests of the resulted parameters live on the simulator it appears that the car oscillated quickly when the speed was increasing. To reduce the oscillations, P value was decreased to 0.5.

I have also implemented a separate PID controller for speed. The similar technique was used to choose the hyperparameters of the one. However, more manual tuning of both steering and speed PID controllers components values was required to make it run safely. To run the code using both PID controllers, `useSpeedPID` in `main.cpp` should be set to `true`.

### Results

The results of steering wheel PID controller implementation can be found in [this video](./videos/lane.mov).

When using both steering and speed PID controllers the car moves not so smooth as in previous case, but it sometimes reaches the speed up to almost 60mph. The video for this case can be found [here](./videos/speed_lane.mov).


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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
