#!/usr/bin/env python3

import subprocess
import os
import signal
import time
import sys
import rospy


# Callback to handle SIGINT and SIGTERM
def keyboard_interrupt_callback(_1, _2):
    sys.exit(0)


if __name__ == '__main__':
    # Allow keyboard exit
    signal.signal(signal.SIGINT, keyboard_interrupt_callback)

    # ROS setup
    rospy.init_node('turtlebot2_share_pd')

    # Continually reopens the comms_turtlebot2_share.pd patch if it closes
    while not rospy.is_shutdown():
        command = 'puredata -open '
        filepath = os.path.join(os.path.dirname(__file__), 'comms_turtlebot2_share.pd')
        subprocess.run(command + filepath, shell=True)
        time.sleep(.2)
