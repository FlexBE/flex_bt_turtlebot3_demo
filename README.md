Flex BT Turtlebot
================================

## Introduction

Implementations of the [ROS 2] and [FlexBE]-based [Flexible Behavior Trees] for use with the [ROBOTIS Turtlebot3].

This repository contains code that demonstrates the open-source ROS 2 [Flexible Behavior Trees] system using the standard [ROBOTIS Turtlebot3]

ROS 2 release.  The demonstrations include installation and setup instructions for the [FlexBE System].

----------------------

This package has a number of dependencies.  The quickest, and easiest method to get a demonstration up and running, is to follow the setup instructions below.  

This quickly sets up the entire system in a separate workspace.

1) Ensure that you are running a recent ROS 2 version; this system is tested on `ros-humble-desktop` under Ubuntu 22.04.  
   See [ROS 2 Installation] for more information.

2) Globally install the [ROBOTIS Turtlebot3] and [ROS 2 Navigation2] packages and their dependencies

3) Install the [FlexBE App], [FlexBE System], and [Flexible Behavior Trees] into a ROS 2 workspace

4) Make sure the [FlexBE App] is properly set up following directions there

 This version presumes use of the [FlexBE App] for the operator interface, which depends on states and behaviors that are exported as part of individual package.xml.


## Operation
---------

The following directions are for a simple demonstration using the Turtlebot3 simulations that are standard with Navigation 2 in ROS 2 releases.

### Start the simulated robot
`export TURTLEBOT3_MODEL=burger`
 * This defines which version of the Turtlebot3 will be simulated

`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`
 * Starts the simulated environment of the ROBOTIS Turtlebot3 world with the simulated Turtlbot3

### Start localization

`export TURTLEBOT3_MODEL=burger`
 * Need to export the Turtlebot3 model variable again in the separate terminal tab

 Start one and only one of the following.

 We have tested with cartographer and slam toolbox using an unknown map and using AMCL.

 `ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True`
  * Starts ROS 2 Cartographer to build a map
  * Sets use_sim_time to true instead of the robot running in real time

or

`ros2 launch nav2_bringup slam_launch.py use_sim_time:=True`
  * Does SLAM using the Nav2 Slam Toolbox setup

or

 `ros2 launch nav2_bringup localization_launch.py use_sim_time:=True map:=/opt/ros/$ROS_DISTRO/share/nav2_bringup/maps/turtlebot3_world.yaml`
   * Starts AMCL using the defined map matching the world in Gazebo simulation
   * You need to set the Initial Pose using RViz as described below

   > From Nav2 instructions: After starting, the robot initially has no idea where it is. By default, Nav2 waits for you to give it an approximate starting position. Take a look at where the robot is in the Gazebo world, and find that spot on the map. Set the initial pose by clicking the “2D Pose Estimate” button in RViz, and then down clicking on the map in that location. You set the orientation by dragging forward from the down click.

   > Note: 30-June-22 Humble release did not update static map layer, which prevented planning with AMCL.  Disable static layer in params/nav2_turtlebot_params.yaml if you have issues with AMCL.


### Visualization

  `ros2 launch flex_bt_turtlebot_demo_bringup rviz.launch.py  use_sim_time:=True`

  > NOTE: With AMCL localization, RViz and localization may generate errors until the "2D Pose Estimate" is set via RViz.

  > NOTE: The simulation automatically starts RViz alongside the ROS 2 Cartographer command; verify topic settings as needed if using that version.


### Startup of Navigation2

Navigation2 Stack requires startup of planning and control nodes.

`ros2 launch flex_bt_turtlebot_demo_bringup nav2_turtlebot.launch.py use_sim_time:=True`
 * This starts the Navigation2 Stack with behavior server
 * Set use_sim_time to True for the simulation demonstration
 * As this is for a simulation demo, the parameter use_sim_time:=True

### Startup FlexBE in simulation

`ros2 launch flexbe_app flexbe_full.launch.py use_sim_time:=True`
  * Starts both the FlexBE app and FlexBE behavior engine

or,

start Operator Control Station (OCS) UI and onboard separately:

`ros2 launch flexbe_app flexbe_ocs.launch.py use_sim_time:=True`
  * Starts the FlexBE app on OCS computer

`ros2 launch flexbe_onboard behavior_onboard.launch.py use_sim_time:=True`
  * Starts the FlexBE behavior engine for robot control


 >NOTE: after a new build , the Node JS software must be downloaded and installed before the first run.

  `ros2 run flexbe_app nwjs_install`

> This is only needed before first run after a fresh build (e.g. if you delete the install folder)



### FlexBE Operation

After startup, all control is through the FlexBE App operator interface and RViz.  

First load the desired behavior through the `FlexBE Behavior Dashboard` tab.
  * `Turtlebot Nav2 BT`
     * Simplest example allows user to input goal via FlexBE state and RViz

  * `Turtlebot Nav2 Multi-BTs`
     * Basic navigation using multiple separate BTs

  * `Turtlebot Patrol`
     * Allows user to input location of charging station and multiple waypoints to patrol using RVIZ.
     * Patrols and periodically moves to recharge station
     * This uses a battery status topic.  A simple simulated battery drain and charge can be run with:
       * `ros2 launch flex_bt_turtlebot_demo_bringup turtlebot_sim_battery.launch.py use_sim_time:=True `
       * The FlexBE app will report missing data if the battery status is not running.

Execute the behavior via the `FlexBE Runtime Control` tab.
* The system requires the operator to input a `2D Nav Goal` via the `RViz` screen
  * If the system is in `low` autonomy or higher, the system will request a global plan as soon as the goal is received
  * If the autonomy level is `off`, then the operator will need to confirm receipt by clicking the `done` transition.
* After requesting a path to the goal, the resulting plan will be visualized in the `RViz` window.  
  * If the system is not in full autonomy mode, the operator must confirm that the system should execute the plan via the `FlexBE UI`  
  * If the operator sets the `Runtime Executive` to `full` autonomy, the plan will automatically be executed.  
  * In less than `full` autonomy, the operator can request a recovery behavior at this point.
* Once execution of this plan is complete, `FlexBE` will seek permission to continue planning
  * In `full` autonomy, the system will automatically transition to requesting a new goal
  * In any autonomy level less than `full`, the system will require an operator decision to continue

Whenever a plan is being executed, the `FlexBE` state machine transitions to a concurrent node that uses on line  planners to refine the plans as the robot moves, and also monitors the Turtlebot bumper status for collision.  The operator can terminate the execution early by selecting the appropriate transition in the `FlexBE UI`.  If this low level plan fails, the robot will request permission to initiate a recovery behavior; in `full` autonomy the system automatically initiates the recovery.

---

## Publications

Please use the following publication for reference when using Flexible Behavior Trees:

- Joshua M. Zutell, David C. Conner, and Philipp Schillinger, ["Flexible Behavior Trees: In search of the mythical HFSMBTH for Collaborative Autonomy in Robotics"](https://doi.org/10.48550/arXiv.2203.05389), March 2022.


### Further Publications for FlexBE

- Joshua Zutell, David C. Conner and Philipp Schillinger, ["ROS 2-Based Flexible Behavior Engine for Flexible Navigation ,"](http://dx.doi.org/10.1109/SoutheastCon48659.2022.9764047), IEEE SouthEastCon, April 2022.

- Philipp Schillinger, Stefan Kohlbrecher, and Oskar von Stryk, ["Human-Robot Collaborative High-Level Control with Application to Rescue Robotics"](http://dx.doi.org/10.1109/ICRA.2016.7487442), IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, May 2016.

- Stefan Kohlbrecher et al. ["A Comprehensive Software Framework for Complex Locomotion and Manipulation Tasks Applicable to Different Types of Humanoid Robots."](http://dx.doi.org/10.3389/frobt.2016.00031) Frontiers in Robotics and AI 3 (2016): 31.

- Alberto Romay et al., [“Collaborative autonomy between high-level behaviors and human operators for remote manipulation tasks using different humanoid robots,”](http://dx.doi.org/10.1002/rob.21671) Journal of Field Robotics, September 2016.

---

[FlexBE]: https://flexbe.github.io
[FlexBE App]: https://github.com/FlexBE/flexbe_app
[FlexBE System]: https://github.com/FlexBE/flexbe_behavior_engine
[Flexible Behavior Trees]: https://github.com/FlexBE/flexible_behavior_trees
[ROBOTIS Turtlebot3]: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
[ROS 2]: https://docs.ros.org/en/foxy/index.html
[ROS 2 Installation]: https://docs.ros.org
[ROS 2 Navigation2]: https://navigation.ros.org/
[ROS 2 Cartographer]: https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html
