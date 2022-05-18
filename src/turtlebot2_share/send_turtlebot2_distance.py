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
    if connected:
        pd_socket.close()
    sys.exit(0)


# Sends the minimum detected distance to Pd
def distance_to_pd(data):
    if connected:
        msg_distance = 'distance ' + str(min(data.ranges)) + ';'
        print(msg_distance)
        pd_socket.send(msg_distance.encode('utf-8'))


if __name__ == '__main__':
    # Check for appropriate args (there are 3 default args from ROS)
    if len(sys.argv) == 5:
        ip = sys.argv[1]
        port = int(sys.argv[2])
    else:
        print("Two arguments (IP and port) should be provided")
        sys.exit(1)

    # Allow keyboard exit
    signal.signal(signal.SIGINT, keyboard_interrupt_callback)

    # ROS setup
    rospy.init_node('send_turtlebot2_distance', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, distance_to_pd)

    connected = False

    # In loop for persistence
    while not rospy.is_shutdown():
        # Establish local loopback connection to Pd comms patch
        pd_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (ip, port)
        print(os.path.basename(__file__) + f' starting up on {server_address[0]} port {server_address[1]}')

        try:
            # If successful in connecting, the subscriber/publisher will handle the rest
            pd_socket.connect(server_address)
            print(os.path.basename(__file__) + ' connected!')
            connected = True
            rospy.spin()
        except (ConnectionRefusedError, BrokenPipeError):
            # ConnectionRefusedError rises if the ROS-to-Pd node attempts connection before the Pd comms patch is ready
            # BrokenPipeError rises if the Pd comms patch dies or is stopped while the ROS-to-Pd node is connected
            print(os.path.basename(__file__) + ' disconnected, reconnecting...')
            connected = False
            # Avoid trying too frequently
            time.sleep(.2)
        finally:
            pd_socket.close()
