# SonifyIt

SonifyIt is a sound generation system for robots. By interfacing [Robot Operating System (ROS)](https://wiki.ros.org/) and [Pure Data (Pd)](https://puredata.info/), roboticists and sound designers can collaborate to develop, deploy, and share sound designs for robots. 

Currently, SonifyIt is a working example of two sound designs for the TurtleBot 2, rather than a full package. The package has been released in this form to encourage other researchers to begin experimenting with the system, providing feedback for its eventual form. We plan on developing this package further, adding tools, architectural rules, and more examples.

For further information on how SonifyIt works, please read the README files in each folder.

If you have any questions, are interested in contributing, or just would like to read more about this work, please contact me via my website: <https://www.brianzhang.org/contact-me/>

## Usage

To use the package as-is, you will need:

1. A computer with some flavor of Ubuntu 20.04 and [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
2. A TurtleBot 2
3. An RPLIDAR A1

Parts 2 and 3 can be simulated. 

You will need to install Pure Data (`sudo apt install puredata`) and the following TurtleBot 2 related packages, which you can install via the commands below:

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

Next, you can install this package:

```
cd ~/catkin_ws/src
git clone https://github.com/shareresearchteam/sonify_it.git
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
roslaunch turtlebot2_share.launch
```

You may want to teleoperate the robot, which you can do with:

```
roslaunch turtlebot_teleop keyboard_teleop.launch
```

