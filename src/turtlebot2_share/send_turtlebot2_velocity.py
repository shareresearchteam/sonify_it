#!/usr/bin/env python3

import signal
import sys
import os
import time
import socket
import rospy

from geometry_msgs.msg import Twist


# Callback to handle SIGINT and SIGTERM
def keyboard_interrupt_callback(_1, _2):
    if connected:
        pd_socket.close()
    sys.exit(0)


# Send linear and angular velocity to Pd
def motion_to_pd(data):
    if connected:
        msg_linear_velocity = 'linear_velocity ' + str(data.linear.x) + ';'
        pd_socket.send(msg_linear_velocity.encode('utf-8'))

        msg_angular_velocity = 'angular_velocity ' + str(data.angular.z) + ';'
        pd_socket.send(msg_angular_velocity.encode('utf-8'))
        print(msg_linear_velocity)


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
    rospy.init_node('pd_transport', anonymous=True)
    rospy.Subscriber('/mobile_base/commands/velocity', Twist, motion_to_pd)

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
            connected = True
            print(os.path.basename(__file__) + ' connected!')
            rospy.spin()
        except (ConnectionRefusedError, BrokenPipeError):
            connected = False
            # ConnectionRefusedError rises if the ROS-to-Pd node attempts connection before the Pd comms patch is ready
            # BrokenPipeError rises if the Pd comms patch dies or is stopped while the ROS-to-Pd node is connected
            print(os.path.basename(__file__) + ' disconnected, reconnecting...')
            # Avoid trying too frequently
            time.sleep(.2)
        finally:
            pd_socket.close()
