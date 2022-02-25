#!/usr/bin/env python3

import signal
import sys
import os
import time
import socket
import rospy

from sensor_msgs.msg import LaserScan


# Callback to handle SIGINT and SIGTERM
def keyboard_interrupt_callback(_1, _2):
    sys.exit(0)


# Sends the minimum detected distance to Pd
def distance_to_pd(data):
    if connected:
        msg_distance = 'distance ' + str(min(data.ranges)) + ';'
        print(msg_distance)
        sock.send(msg_distance.encode('utf-8'))


if __name__ == '__main__':
    # Allow keyboard exit
    signal.signal(signal.SIGINT, keyboard_interrupt_callback)

	# ROS setup
    rospy.init_node('send_turtlebot2_distance', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, distance_to_pd)

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
