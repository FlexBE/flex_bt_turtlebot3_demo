Flex BT Turtlebot
================================

## Introduction

CHRISLab specific implementations of the [ROS 2] and [FlexBE]-based [Flexible Behavior Trees] for use with the ROBOTIS-based Turtlebot3 and the Kobuki Turtlebot2.

This repository contains code that demonstrates the open-source CHRISLab ROS 2 [Flexible Behavior Trees] system.  There are interfaces with the [ROBOTIS Turtlebot3] and Kobuki Turtlebot2, as well as installation and setup instructions for the [FlexBE System].

----------------------

This package has a number of dependencies.  The quickest, and easiest method to get a demonstration up and running, is to follow the setup instructions below.  

This quickly sets up the entire system in a separate workspace.

1) Ensure that you are running a recent ROS 2 version; this system is tested on `ros-foxy-desktop-full`.  See [ROS 2 Installation] for more information.

2) Globally install the [ROBOTIS Turtlebot3], [ROS 2 Navigation2], and [urg_node] packages and their dependencies

3) Install the Turtlebot2 [Kobuki ROS] and [Koubki ROS Interfaces] along with the [laser filters] packages into a ROS 2 workspace

3) Install the [FlexBE App], [FlexBE System], and [Flexible Behavior Trees] into a ROS 2 workspace

4) Make sure the [FlexBE App] is properly set up.

 This version presumes use of the [FlexBE App] for the operator interface, which depends on states and behaviors that are exported as part of individual package.xml.


## Operation
---------

The following directions are for a simple demonstration using [ROS 2 Cartographer] in simulation with the Turtlebot3 and on hardware with the Turtlebot2.

### Start the simulated robot
`export TURTLEBOT3_MODEL=burger`
 * This defines which version of the Turtlebot3 will be simulated

`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`
 * Starts the simulated environment of the ROBOTIS Turtlebot3 world with the simulated Turtlbot3

 or

### Start the hardware
`sudo chmod a+rw /dev/ttyACM0`
* This allows for the Hokuyo LiDAR on the Turtlebot2 to get LiDAR data

`ros2 launch flex_bt_turtlebot_demo_bringup turtlebot_bringup.launch.py`
* Starts the hardware on the Turtlebot2  

### Start map server in simulation
 With an unknown map, ROS 2 Cartographer will build a map of the simulated environment
 `export TURTLEBOT3_MODEL=burger`
  * Need to export the Turtlebot3 model variable again in the separate terminal tab

 `ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True`
  * Starts ROS 2 Cartographer to build a map
  * Sets use_sim_time to true instead of the robot running in real time

or

### Start map server on the hardware
`ros2 launch flex_bt_turtlebot_demo_bringup view_model.launch.py`
* Gets the transforms for the Turtlebot2

`ros2 launch flex_bt_turtlebot_demo_bringup turtlebot_cartographer.launch.py`
* Starts ROS 2 Cartographer to build a map

### Visualization
 The simulation automatically starts RViz alongside the ROS 2 Cartographer command
 Displays a standard view of transforms of Turtlebot3, sensor data, with maps, and paths displayed

 For the hardware demo, run on a separate computer connected to the Turtlebot2:
  `ros2 launch flex_bt_turtlebot_demo_bringup turtlebot_rviz.launch.py`

### Startup of Navigation2
Navigation2 Stack requires startup of planning and control nodes.
 `ros2 launch flex_bt_turtlebot_demo_bringup nav2.launch.py`
 * This starts the Navigation2 Stack
 * Set use_sim_time to True for the simulation and false for the hardware
 * For the simulation demo, add the parameter use_sim_time:=True
   `ros2 launch flex_bt_turtlebot_demo_bringup nav2.launch.py use_sim_time:=True`

### Startup FlexBE in simulation
`ros2 launch flexbe_app flexbe_full.launch.py`
* Starts both the FlexBE app and FlexBE behavior engine

or

### Startup FlexBE on hardware
On a separate computer connected to the Turtlebot2
 `ros2 launch flexbe_app flexbe_ocs.launch.py`
 * Starts the FlexBE app

On the Turtlebot2
 `ros2 launch flexbe_onboard behavior_onboard.launch.py`
 * Starts the FlexBE behavior engine



### FlexBE Operation
After startup, all control is through the FlexBE App operator interface and RViz.  
* First load the desired behavior through the `FlexBE Behavior Dashboard` tab.
  * `Turtlebot Nav2 BT`
  * `Turtlebot Nav2 Multi-BTs`
* Execute the behavior via the `FlexBE Runtime Control` tab.
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

[FlexBE]: https://flexbe.github.io
[FlexBE App]: https://github.com/CNURobotics/flexbe_app
[FlexBE System]: https://github.com/CNURobotics/flexbe_behavior_engine
[Flexible Behavior Trees]: https://github.com/CNURobotics/flexible_navigation
[Kobuki ROS]: https://github.com/kobuki-base/kobuki_ros
[Koubki ROS Interfaces]: https://github.com/kobuki-base/kobuki_ros_interfaces
[laser filters]: https://github.com/ros-perception/laser_filters
[ROBOTIS Turtlebot3]: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
[ROS 2]: https://docs.ros.org/en/foxy/index.html
[ROS 2 Installation]: https://docs.ros.org/en/foxy/Installation.html
[ROS 2 Navigation2]: https://navigation.ros.org/
[ROS 2 Cartographer]: https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html
[urg_node]: https://github.com/ros-drivers/urg_node
