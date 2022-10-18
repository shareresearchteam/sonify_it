![example workflow](https://github.com/shareresearchteam/sonify_it/actions/workflows/main.yml/badge.svg)

# SonifyIt

SonifyIt is a sound generation system for robots. By interfacing [Robot Operating System (ROS)](https://wiki.ros.org/) and [Pure Data (Pd)](https://puredata.info/), roboticists and sound designers can collaborate to develop, deploy, and share sound designs for robots. 

SonifyIt acts as a bridge between ROS and Pd such that curated data streams available in ROS can be transferred to Pd. These data streams may then be used directly for sonification or to trigger preset sound files or sound generation patches. Unlike classic methods of producing sounds on robots, Pd allows on-the-go modification of designed sounds, greater variability/less repetitiveness, and a more accessible interface for non-programmers. Here is the provided working example of SonifyIt in action:

https://user-images.githubusercontent.com/8885230/169638825-85df6338-d71d-49cc-bc0b-8de52679f526.mp4

For more details on the structure and use of SonifyIt for new robots and associated sound designs, please see [**the wiki**](https://github.com/shareresearchteam/sonify_it/wiki).

If you have any questions, are interested in contributing, or just would like to read more about this work, please contact me via my website: <https://www.brianzhang.org/contact-me/>

## Usage

This package currently contains an example implementation for the Turtlebot 2 robot running [ROS 1 Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu). It is a Python-based package. A [version based on C++](https://github.com/shareresearchteam/sonify_it/tree/noetic_cpp) and a [version for ROS 2](https://github.com/shareresearchteam/sonify_it2) are planned. 

To use the package as-is, you will need:

1. A computer with some flavor of Ubuntu 20.04 and ROS 1 Noetic
2. A TurtleBot 2 (optional; you can simulate one instead)
3. An RPLIDAR A1 (optional; you can simulate an alternative instead)

You will need to install the following TurtleBot 2 related packages, which you can install via the commands below:

```
cd ~/catkin_ws/src # Or your ROS workspace name

# Manually add packages that have not been added to apt for Noetic
git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_simulator.git
git clone https://github.com/turtlebot/turtlebot_apps.git
git clone https://github.com/turtlebot/turtlebot_msgs.git
git clone https://github.com/yujinrobot/kobuki.git
git clone https://github.com/yujinrobot/yujin_ocs.git
git clone https://github.com/yujinrobot/yocs_msgs.git

rm -rf yujin_ocs/yocs_ar* # Remove deprecated packages that won't build

# Install dependencies and build
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

Next, you can install this package and Pure Data:

```
cd ~/catkin_ws/src
git clone https://github.com/shareresearchteam/sonify_it.git
sudo apt install puredata
```

Lastly, build and source your ROS workspace:
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

You can now run one of the following sets of commands (in different windows)

To bring up a real robot:

```
roslaunch turtlebot_bringup turtlebot_bringup.launch
roslaunch sonify_it turtlebot2_share.launch
```

To bring up a simulated robot:

```
roslaunch turtlebot_stage turtlebot_in_stage.launch
roslaunch sonify_it turtlebot2_share.launch
```

You may want to teleoperate the robot, which you can do with:

```
roslaunch turtlebot_teleop keyboard_teleop.launch
```

## Creating Your Own

Please see [the tutorial page](https://github.com/shareresearchteam/sonify_it/wiki/Tutorials) for information on how to extend this package for different robots and new sound designs.
