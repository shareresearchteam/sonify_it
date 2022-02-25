#!/usr/bin/env python3

import signal
import sys
import os
import time
import socket
import rospy

from actionlib_msgs.msg import GoalStatusArray


# Callback to handle SIGINT and SIGTERM
def keyboard_interrupt_callback(_1, _2):
    sys.exit(0)


# Interpret the navigation status, sending:
#    1 if the robot has not moved yet
#    0 if the robot is headed towards the goal
#    -1 for all other cases (e.g. robot reached the goal)
def check_status(data):
    status_list = data.status_list
    if status_list:
        if status_list[0].status == 1:
            return 0
        else:
            return -1
    else:
        return 1

# Send the navigation status to Pd
def status_to_pd(data):
    if connected:
        msg_status = 'status ' + str(check_status(data)) + ';'
        print(msg_status)
        sock.send(msg_status.encode('utf-8'))


if __name__ == '__main__':
    # Allow keyboard exit
    signal.signal(signal.SIGINT, keyboard_interrupt_callback)

	# ROS setup
    rospy.init_node('pd_transport', anonymous=True)
    rospy.Subscriber('/move_base/status', GoalStatusArray, status_to_pd)

	# In loop for persistence
    while not rospy.is_shutdown():
		# Establish local loopback connection to Pd comms patch
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = ('localhost', 9001)
        print(os.path.basename(__file__) + f' starting up on {server_address[0]} port {server_address[1]}')

        try:
			# If successful in connecting, the subscriber/publisher will handle the rest
            sock.connect(server_address)
            print(os.path.basename(__file__) + ' connected!')
            rospy.spin()
        except (ConnectionRefusedError, BrokenPipeError):
            # ConnectionRefusedError rises if the ROS-to-Pd node attempts connection before the Pd comms patch is ready
            # BrokenPipeError rises if the Pd comms patch dies or is stopped while the ROS-to-Pd node is connected
            print(os.path.basename(__file__) + ' disconnected, reconnecting...')
			# Avoid trying too frequently
            time.sleep(.2)
        finally:
            sock.close()
