#!/usr/bin/env python3
"""Listen to relevant tf transforms and publish a map as a Python dict.

We only publish ground plane coordinates x, y in robot frame (/rig).

The dict is encoded as a string with JSON. A proper ROS message would be better, but some learning components
require a UDP interface.

The map is published both to a ROS topic and to a UDP port."""
import tf.transformations as ts
import roslib
import rospy
import tf
import json

import numpy as np
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
    dict_map_frame = dict()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        message = dict()

        # Update required transforms
        for obj in ['/rig', '/board0', '/board1', '/board2', '/board3', '/obstacle1', '/obstacle2', '/obstacle3', '/goal']:
            try:
                (trans,rot) = listener.lookupTransform('/map', obj, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            dict_map_frame[obj] = dict()
            dict_map_frame[obj]["trans"] = trans
            dict_map_frame[obj]["rot"] = rot

        # Build map in rig frame and save them to the map message
        message = dict()
        if '/rig' not in dict_map_frame:
            continue

        trans_robot = dict_map_frame['/rig']['trans']
        rot_robot = dict_map_frame['/rig']['rot']
        rig_transform = ts.concatenate_matrices(ts.translation_matrix(trans_robot), ts.quaternion_matrix(rot_robot))
        inv_rig = np.array(ts.inverse_matrix(rig_transform))

        # Put all other transforms in robot frame
        for key, value in dict_map_frame.items():
            if key is not '/rig':
                trans = value['trans']
                rot = value['rot']
                transform = ts.concatenate_matrices(ts.translation_matrix(trans), ts.quaternion_matrix(rot))
                transform = np.array(transform)
                transform_rig_frame = inv_rig @ transform
                trans = ts.translation_from_matrix(transform_rig_frame)
                rot = ts.quaternion_from_matrix(transform_rig_frame)

            message[key] = [trans[0], trans[1]]


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
