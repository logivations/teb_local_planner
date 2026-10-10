teb_local_planner ROS Package
=============================

The teb_local_planner package implements a plugin to the base_local_planner of the 2D navigation stack. 
The underlying method called Timed Elastic Band locally optimizes the robot's trajectory with respect to trajectory execution time, 
separation from obstacles and compliance with kinodynamic constraints at runtime.

Refer to http://wiki.ros.org/teb_local_planner for more information and tutorials.

Build status of the *melodic-devel* branch:
- ROS Buildfarm (Melodic): [![Melodic Status](http://build.ros.org/buildStatus/icon?job=Mdev__teb_local_planner__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__teb_local_planner__ubuntu_bionic_amd64/)

### Port to ROS2
This branch is the teb_local_planner package ported to ROS2(Dashing Diademata). Currently, it is currently compatible with [Navigation2(master branch)](https://github.com/ros-planning/navigation2/tree/master)([226f06c](https://github.com/ros-planning/navigation2/commit/226f06ce282c727ca240ce8be0cb4b093e26343b)). You can test teb_local_planner with Navigation2 and TurtleBot3 simulation by launching the following command.
```
ros2 launch teb_local_planner teb_tb3_simulation_launch.py
```

## Control latency compensation (Logivations)

TEB plans every cycle from the measured pose and velocity. If the drive needs some time until a
command shows up in the measured motion, the robot keeps executing the commands of that time while
TEB already plans the next one. Near a goal it keeps rolling while TEB brakes, the remaining distance
shrinks faster than planned and every cycle asks for a harder deceleration than `acc_lim_x` (a soft
constraint), so the robot reaches the goal fast or overshoots and oscillates around it.

With `control_latency_s > 0` TEB keeps the commands the drive received and plans from the state
predicted for the moment the new command takes effect (`ControlLatencyPredictor`, a Smith predictor
with a dead-time model): the measured pose is integrated over `control_latency_s` with the commands
of the last `control_latency_s`. With `control_lag_s = 0` the newest command is the start velocity
of the plan; with `control_lag_s > 0` the speed follows the commands with that first-order lag after
the dead time, starting from the measured speed, and the plan starts with the speed the model reaches.
The predicted pose is used for everything inside TEB (plan matching, via-points, obstacles, the plan
start); the goal and progress checkers of the controller server keep using the measured pose.

Which commands: set `control_latency_executed_cmd_topic` to the TwistStamped topic the drive gets
(after the smoother / collision monitor and every other filter between TEB and the drive). Without
it TEB predicts with its own commands, which is only right if nothing downstream changes them: when
the downstream zeroes or caps them, the prediction runs away from the robot and TEB's own last
command becomes its next start velocity (amr47 2026-10-10: plan start up to 12 cm off the standing
robot, TEB flipping forward/reverse every cycle, wheel demand -86 <-> +89 deg, 17 s without traction;
while driving 4x larger command steps).

| Parameter | Default | Meaning |
|---|---|---|
| `control_latency_s` | 0.0 | dead time [s] from a command to the drive's reaction (with `control_lag_s` 0: to the measured motion); 0 disables the compensation |
| `control_lag_s` | 0.0 | time constant [s] of the drive's first-order speed response after the dead time; 0 = pure dead time |
| `control_latency_executed_cmd_topic` | "" | TwistStamped topic of the commands the drive executes; empty = TEB's own commands. Read at startup |
| `control_latency_min_speed` | 0.05 | no prediction while the measured speed and the newest command are below this [m/s] |
| `steering_latency_s` | 0.0 | car-like steering model: dead time [s] from a command to the wheel angle |
| `steering_rate_limit` | 0.0 | car-like steering model: max. wheel angle rate [rad/s], 0 = unlimited |
| `control_latency_max_velocity_error` | 0.3 | if the measured speed differs from the command sent `control_latency_s` ago by more than this [m/s], the prediction is skipped for that cycle |

The steering model (tricycle, `omega = v * tan(delta) / wheelbase`) is active when `wheelbase > 0`
and `steering_latency_s` or `steering_rate_limit` is set: the wheel angle follows the commanded one
with the dead time and the rate limit, and the predicted turn rate follows the wheel. Without it the
predicted turn rate keeps the commanded curvature.

The prediction is skipped (measured state, as without compensation) when no command was sent within
the last 0.5 s (controller idle, another controller drove), after `reset()`, when the robot does
not follow the commands (collision monitor stop, speed limits downstream of TEB, traction stuck), see
`control_latency_max_velocity_error`, at standstill (`control_latency_min_speed`), and, with a
wheelbase, when the commands or the measured motion of the window change between forward and reverse
at a wheel angle beyond 45 deg (the wheel turns to the other side before the drive moves).

## Citing the Software

*Since a lot of time and effort has gone into the development, please cite at least one of the following publications if you are using the planner for your own research:*

- C. Rösmann, F. Hoffmann and T. Bertram: Integrated online trajectory planning and optimization in distinctive topologies, Robotics and Autonomous Systems, Vol. 88, 2017, pp. 142–153.
- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Trajectory modification considering dynamic constraints of autonomous robots. Proc. 7th German Conference on Robotics, Germany, Munich, May 2012, pp 74–79.
- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Efficient trajectory optimization using a sparse model. Proc. IEEE European Conference on Mobile Robots, Spain, Barcelona, Sept. 2013, pp. 138–143.
- C. Rösmann, F. Hoffmann and T. Bertram: Planning of Multiple Robot Trajectories in Distinctive Topologies, Proc. IEEE European Conference on Mobile Robots, UK, Lincoln, Sept. 2015.
- C. Rösmann, F. Hoffmann and T. Bertram: Kinodynamic Trajectory Optimization and Control for Car-Like Robots, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Vancouver, BC, Canada, Sept. 2017.

<a href="https://www.buymeacoffee.com/croesmann" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/lato-black.png" alt="Buy Me A Coffee" height="31px" width="132px" ></a>

## Videos

The left of the following videos presents features of the package and shows examples from simulation and real robot situations.
Some spoken explanations are included in the audio track of the video. 
The right one demonstrates features introduced in version 0.2 (supporting car-like robots and costmap conversion). Please watch the left one first.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=e1Bw6JOgHME" target="_blank"><img src="http://img.youtube.com/vi/e1Bw6JOgHME/0.jpg" 
alt="teb_local_planner - An Optimal Trajectory Planner for Mobile Robots" width="240" height="180" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=o5wnRCzdUMo" target="_blank"><img src="http://img.youtube.com/vi/o5wnRCzdUMo/0.jpg" 
alt="teb_local_planner - Car-like Robots and Costmap Conversion" width="240" height="180" border="10" /></a>

## License

The *teb_local_planner* package is licensed under the BSD license.
It depends on other ROS packages, which are listed in the package.xml. They are also BSD licensed.

Some third-party dependencies are included that are licensed under different terms:
 - *Eigen*, MPL2 license, http://eigen.tuxfamily.org
 - *libg2o* / *g2o* itself is licensed under BSD, but the enabled *csparse_extension* is licensed under LGPL3+, 
   https://github.com/RainerKuemmerle/g2o. [*CSparse*](http://www.cise.ufl.edu/research/sparse/CSparse/) is included as part of the *SuiteSparse* collection, http://www.suitesparse.com. 
 - *Boost*, Boost Software License, http://www.boost.org

All packages included are distributed in the hope that they will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the licenses for more details.

## Requirements

Install dependencies (listed in the *package.xml* and *CMakeLists.txt* file) using *rosdep*:

    rosdep install teb_local_planner


