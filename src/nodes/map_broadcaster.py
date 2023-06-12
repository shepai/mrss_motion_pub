#!/usr/bin/env python3
"""Listen to relevant tf transforms and publish a map as a Python dict.

We only publish ground plane coordinates x, y in robot frame (/rig).

The dict is encoded as a string with JSON. A proper ROS message would be better, but some learning components
require a UDP interface.

The map is published both to a ROS topic and to a UDP port."""
import roslib
import rospy
import tf
import json

import socket
import sys
import json
from std_msgs.msg import String

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

if __name__ == '__main__':
    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP

    rospy.init_node('map_broadcaster')
    listener = tf.TransformListener()
    map_pub = rospy.Publisher('map', String, queue_size=10)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        message = dict()

        # Lookup required transforms
        for obj in ['/board0','/board1','/board2','/board3']:
            # TODO BEGIN MRSS: Look up a relevant transform
            try:
                (trans,_) = listener.lookupTransform('/rig', obj, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            # End MRSS

            # TODO BEGIN MRSS: Add transform to Dict
            message[obj] = [trans[0], trans[1]]
            # End MRSS


        # Add timestamp
        time = rospy.Time.now()
        message["time_sec"] = time.to_sec()
        message["time_nsec"] = time.to_nsec()

        message_str = json.dumps(message)

        # Send to UDP
        try:
            # Connect to server and send data
            sock.sendto(bytes(message_str, encoding="utf-8"), (UDP_IP, UDP_PORT))
        except Exception as e:
            rospy.logerr(e)

        # Send as a ROS message
        map_pub.publish(message_str)

        rate.sleep()

    # Close socket
    sock.close()
