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

## Creating Your Own

In its current state, each implementation of SonifyIt will more like implementing a template than extending a package. We hope to implement several features to this repository that will make the process better, including:

- Pd subpatches and externals, which will provide templates for common sonification methods.
- ROS-to-Pd nodes for common message types and desired data features, such as the presence of a human face. 
- ROS services that help control the activity of SonifyIt. 
- Instructions on how to use SonifyIt with a headless robot for both the prototyping (e.g. externals that allow remote access to ROS) and deployment phases (e.g. reliable subprocess calls for starting Pd headlessly with desired audio settings). 

For the time being, this is how you can set up your own SonifyIt implementation for a generic robot with a desktop environment, such as through TeamViewer, VNC, or other desktop remote access options.

1. Identify actions, states, or other situations you would like the robot to sonify (e.g. a person is detected, activating sounds like acknowledgement or warning). 
2. For each of the above, select a ROS topic or data within a ROS topic that is connected (e.g. we want to know how far away the closest person is; `\people_tracker_measurements` from `leg_detector`could help).
3. Write ROS-to-Pd nodes that calculate the relevant information and sends it to Pd. The format of the message is a string that contains: "`variable_name data`". Data as single floats or integers are easiest, but more complex messages are possible to unpack as well. See the `send_<something>.py` files for examples. 
4. Copy `pd_turtlebot2_share.py` and `comms_turtlebot2_share.pd`, renaming the `turtlebot2_share` parts as desired for your robot and tag. 
5. In `pd_<your_robot>_<your_tag>.py` (previously `pd_turtlebot2_share.py`), update all mentions of `turtlebot2_share` as in step 4. 
6. In `comms_<your_robot>_<your_tag>.pd` (now `comms_turtlebot2_share.pd`), change the contents of the `route` object to include all of the variable names established in step 3. 
7. Create a `controller_<your_robot>_<your_tag>.pd` file with the same number of inlets as variables established in step 3. In `comms_<your_robot>_<your_tag>.pd`, change the `controller_turtlebot2_share` object to `controller_<your_robot>_<your_tag>` and draw a connection between the first outlet of the `route` object to the first inlet of the `controller_<your_robot>_<your_tag>`, the second outlet to the second inlet, and so on. There should be one disconnected outlet (essentially the `else` outlet) on the `route` object at the end. 
8. In your `controller_<your_robot>_<your_tag>`, try things out! Pure Data is very powerful for creating different generative sounds and the design space is wide open for experimentation. For some comparisons of sound design modalities, I recommend:
[Robot Gesture Sonification to Enhance Awareness of Robot Status and Enjoyment of Interaction](https://ieeexplore.ieee.org/document/9223452), Zahray et al.
[Smooth Operator: Tuning Robot Perception Through Artificial Movement Sound](https://dl.acm.org/doi/abs/10.1145/3434073.3444658), Robinson et al. 
9. Change your launch file to include all of your `send_<something>` nodes and your `pd_<your_robot>_<your_tag>.py` node.
10. Launch and experiment with your sonification live!
