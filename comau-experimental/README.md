# Comau Experimental

## Table of Contents

- [Comau Experimental](#comau-experimental)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Installation](#installation)
    - [Building from Source](#building-from-source)
      - [Requirements](#requirements)
      - [Building procedure](#building-procedure)
  - [How to use the COMAU ROS](#how-to-use-the-comau-ros)
    - [Simulation](#simulation)
  - [Real Robot](#real-robot)
    - [After that you are ready to start interfacing with the robot through ros](#after-that-you-are-ready-to-start-interfacing-with-the-robot-through-ros)

## Overview

This repository contains all the required ROS packages to work with Comau robots through ROS.

The Comau Experimental package has been tested under ROS Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Installation

### Building from Source

#### Requirements

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

It is recommended to use **Ubuntu 18.04 with ROS melodic**, however using Ubuntu 16.04 with ROS kinetic should also work.

#### Building procedure

```bash
# source global ros
source /opt/ros/<your_ros_version>/setup.bash

# create a catkin workspace
mkdir -p catkin_ws/src && cd catkin_ws/src

# Clone the latest version of this repository into your catkin workspace *src* folder.
git clone <repository link>

# Install dependencies of all packages.
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build the workspace
catkin_make

# activate the workspace
source devel/setup.bash

```

## How to use the COMAU ROS

### Simulation

Follow the instructions at [comau_sim README](comau_sim/README.md)

## Real Robot

To start the driver follow the instructions at [comau_driver README](comau_driver/README.md)


### After that you are ready to start interfacing with the robot through ros 

1. How to use Asynchronous Joint/Cartesian feature
    
    [comau_action_handlers README](comau_action_handlers/README.md)

2. How to use Sensor Tracking feature 

    [comau_controllers README](comau_controllers/README.md)


<!-- TODO move the following to Comau safe spawner script -->
<!-- ## Comau Safe Spawner Script 

To automate the launch process, a script was implemented that spawns multiple xterm terminals ready with the correct command for your specific case.

```bash
# To Install xterm
sudo apt-get update -y
sudo apt-get install -y xterm
# move to comau_safe_spawner package
roscd comau_safe_spawner
# make the script executable
chmod +x main.sh
# run the script
./main.sh
```

1. The Script will show the available robots and will ask you to choose the associating number
2. It will then ask you if you want to launch :
        a) Gazebo : headless or not
        b) Driver : with RoboShop or Real Configuration
        c) RViz : will **only** launch RViz with joint_state_publisher_gui
3. If you want to launch MoveGroup Node
4. If you want to launch all the Comau Action Servers
5. If you want to start RViz
6. If you want to start the Comau Demo -->
#comau
