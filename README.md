# Obstacle Avoidance on Turtlebot
[![License](https://img.shields.io/badge/License-BSD%203--Clause-orange.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Build Status](https://travis-ci.org/urastogi885/obstacle_avoidance_simulation.svg?branch=master)](https://travis-ci.org/urastogi885/obstacle_avoidance_simulation)

## Overview

ROS package to simulate obstacle avoidance behavior on a turtlebot

## Dependencies

- Ubuntu 16.04
- ROS Kinetic
- Gazebo
- Turtlebot Packages

## Install Dependencies

- This project was developed using ROS Kinetic.
- It is highly recommended that ROS Kinetic is properly installed on your system before the use of this project.
- Follow the instructions on this [*link*](http://wiki.ros.org/kinetic/Installation/Ubuntu) to install ***Full-Desktop 
  Version*** of ROS Kinetic.
- The full-version would help you install *Gazebo* as well. If you have ROS Kinetic pre-installed on your machine, use
  the following [*link*](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) to install *Gazebo* on your
  machine.
- Ensure successful installation by running *Gazebo* via your terminal window:
```shell script
gazebo
```
- A window of *Gazebo Simulator* should be launched.
- Create your ROS workspace by following instructions on this [*link*](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
- Install the necessary turtlebot packages:
```shell script
sudo apt install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```

## Build

- Switch to your *src* sub-directory of your ROS workspace to clone this repository.
```shell script
<ROS Workspace>/src
```
- Run the following commands to clone and build this project:
```shell script
git clone --recursive https://github.com/urastogi885/obstacle_avoidance_simulation
cd ..
catkin_make
```

## Run

- In the same terminal, run:
```shell script
roscore
```
- Open a new terminal, switch to the ROS workspace, and launch the obstacle avoidance node:
```shell script
cd <ROS Workspace>
source devel/setup.bash
roslaunch turtlebot_obstacle_avoidance obstacle_avoidance.launch enableRosBag:=false
```
- A turtlebot world will open up on the Gazebo Simulator and you will be able to see the robot avoiding various
  obstacles in its path.
- Change your camera view to have better look at the movement of the robot.
- Stop execution using *Ctrl+C*.

## Generate/View Rosbag File

- You can either generate a new ROS bag file or play the rosbag file included in the *results* directory.
- To generate a new bag file, launch the obstacle avoidance node. (You can skip this step if you just want to play the
  previously generated bag file)
```shell script
roslaunch turtlebot_obstacle_avoidance obstacle_avoidance.launch enableRosBag:=true
```
- The bag file would have been replaced with new data.
- To play the ROS bag file, make sure nothing is running except *roscore*, open a new terminal window, and run :
```shell script
cd <ROS Workspace>
source devel/setup.bash
cd src/obstacle_avoidance_simulation/worlds
rosrun gazebo_ros gazebo turtlebot_enclosed_world.world
```
- The above commands will help you launch the turtlebot world in Gazebo.
- Open a new terminal, switch to ROS workspace, and run:
```shell script
cd <ROS Workspace>
source devel/setup.bash
cd src/obstacle_avoidance_simulation/results
rosbag play record_obstacle_avoidance.bag
```
- You should be able to see the obstacle avoidance behavior on turtlebot in the Gazebo window.
- You can also verify the bag file by running: (Stop the execution of last command before proceeding any further by
using *Ctrl+C*)
```shell script
rosbag info record_obstacle_avoidance.bag
``` 

## Documents

- These are located in the *results* sub-directory:
    - Cpplint - *cpplint_output.txt*
    - Cppcheck - *cppcheck_output.txt*
    - Rosbag - *record_obstacle_avoidance.bag*
    - RQT Graph Output - *rqt_graph.png*
    - Turtlebot World - *customized_turtlebot_world.png*