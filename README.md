# TurtleBot3 Dynamic Tracking

**turtlebot3_dynamic_tracking** is a ROS 2-based project for controlling a TurtleBot3 robot to dynamically track a reference trajectory using a Model Predictive Path Integral (MPPI) controller. This project demonstrates how to implement dynamic trajectory tracking for mobile robots in environments with changing targets, using ROS 2 Humble and the Nav2 navigation stack.

## Features

- **Dynamic Trajectory Tracking:** The robot can follow a reference trajectory that changes over time.
- **Model Predictive Path Integral (MPPI):** Uses MPPI for optimal control, which computes a control sequence by simulating the system forward in time and optimizing over future trajectories.
- **ROS 2 Integration:** Fully integrates with ROS 2 (Humble) to control the TurtleBot3 robot and visualize the tracking performance.
- **Nav2 Integration:** Uses Nav2 (Navigation 2) for robust autonomous navigation and path-following.
- **Simulated Environment:** Includes simulation support, allowing you to test the algorithm in a simulated Gazebo environment before running it on the physical robot.
- **Open Source Scout Mini + Indro Commander Dataset:**<a href="https://universe.roboflow.com/scout-0ptgg/scoutmini"><img src="https://app.roboflow.com/images/download-dataset-badge.svg"></img></a>
- **Pretrained Computer Vision Model**: <a href="https://universe.roboflow.com/scout-0ptgg/scoutmini/model/"><img src="https://app.roboflow.com/images/try-model-badge.svg"></img></a>

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Usage](#usage)
4. [Project Structure](#project-structure)
5. [Contributing](#contributing)
6. [License](#license)

## Prerequisites

Before running the `turtlebot3_dynamic_tracking` package, ensure you have the following dependencies installed:

- **ROS 2 Humble:** Tested with ROS 2 Humble Hawksbill.
- **TurtleBot3 Packages for ROS 2:** Required to run the TurtleBot3 simulation and control.
- **Nav2 (Navigation 2):** Provides navigation capabilities for the robot.
- **Gazebo:** For simulating the TurtleBot3 and testing the tracking algorithm.
- **Python 3.x:** The MPPI algorithm is implemented in Python. Ensure you have Python 3.x installed.
- **ROS 2 Python Libraries:** Install `rclpy`, `nav2_msgs`, and other related libraries.

### Installing ROS 2 Humble

If you don’t have ROS 2 Humble installed, follow the instructions for installing ROS 2 Humble on Ubuntu from the official [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html).

### Installing TurtleBot3 Packages for ROS 2
Follwo the instruction to intall nessarary turtlebot3 packages from official [Turtlebot3 PC setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup).
Run the simulation, you need to add the simulation package in your workspace [Turtlebot3 simulation guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

## Run the multiple robot follower 
In 'terminal 1'
```bash
ros2 launch turtlebot3_multi_robot gazebo_multi_world.launch.py
```
In 'terminal 2'
```bash
ros2 run turtlebots_aoc_pkg multi_follower
```
* [multi-robot-video](https://youtu.be/8F5dVyIByNg) 
* [two-robots-video](https://youtu.be/urKhxX07TNw)
