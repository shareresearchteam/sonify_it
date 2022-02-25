# /src/turtlebot2_share

This initial demonstration is for the TurtleBot 2 and is provided by the SHARE Lab. 

### send_turtlebot2_<something\>.py

These are the ROS-to-Pd nodes, which process and then send information from ROS topics to Pd. Each is set up very similarly aside from the subscribed topic and processing. We recommend a naming convention of `send_<robot>_<something>`, which will hopefully encourage re-use of ROS-to-Pd nodes when designing different sound profiles for the same robot. 

Currently, the nodes are written in Python. It is more efficient to write these nodes in C++, which will be the norm for the future.

### pd_turtlebot2_share.py

This node ensures that `comms_turtlebot2_share.pd` is always running. When this node is closed, Pd will also close. This node will be effectively the same for each sound profile, so the requested file may end up becoming an argument passed to this node in roslaunch files.

### comms_turtlebot2_share.pd

This patch contains the networking elements of Pd, which accept packets with the FUDI protocol and split them according to their labeled variable name. Because these names must match the labels applied by `send_turtlebot_<something>` nodes, we recommend that the roboticist maintain this patch. 

This patch includes the `controller_turtlebot2_share.pd` patch, which causes it to open when `comms_turtlebot2_share.pd` is open.

### controller_turtlebot2_share.pd

This is the main patch for the sound design aspect. After receiving the variables from `comms_turtlebot2_share.pd` via inlets, it can then contain or call upon any number of subpatches. In this case, the subpatches are included within the file; however, for more complex subpatches and for more portability, it may be best to have each subpatch be its own file. In that case, we recommend a subpatch folder.

Sound programming in Pd is very powerful, but can take some time to learn, as it is still not as easily accessible as modern digital audio workstations (DAWs). We recommend looking closely through Pd's tutorials and this patch for an understanding of how SonifyIt turns data streams into sound. 

In case a sound designer would prefer to design sounds through a DAW of their choise and then use that sample in Pd, you can do so as shown in this patch. We recommend having a samples folder to avoid cluttering this folder. Note that samples can add up significantly in terms of space. 