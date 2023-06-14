#!/usr/bin/env python3
# Reads the map output (see map_broadcaster.py) and publishes twist commands to reach the goal

import numpy as np
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import json
import time
import torch

import unitree_api_wrapper
from unitree_api_wrapper.go1_controller import Go1Controller


class RLController:
    def __init__(self):
        super().__init__()

        # Initialize Node
        rospy.init_node('RLController', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # Init controller
        self.controller = Go1Controller(policy_path="TODO.pt")
        self.controller.connect_and_stand()

        # Initialize subscriber
        self.cmd_sub = rospy.Subscriber('/cmd_vel', geometry_msgs.msg.Twist, self.cmd_callback)
        self.cmd = geometry_msgs.msg.Twist()
        self.last_cmd_time = time.time()
        
        self.rate = rospy.Rate(50)  # Publisher frequency

    def cmd_callback(self, msg):
        self.last_cmd_time = time.time()
        self.cmd = msg

    def spin(self):
        '''
        Spins the node.
        '''
        try:
            while not rospy.is_shutdown():
                lin_x, lin_y, ang_z = self.cmd.linear.x, self.cmd.linear.y, self.cmd.angular.z
                if time.time() - self.last_cmd_time > 0.1:
                    lin_x, lin_y, ang_z = 0., 0., 0.

                # Run policy
                state, obs, action = self.controller.control_highlevel(torch.Tensor([lin_x, lin_y, ang_z]))
                
                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down controller.")

    def on_shutdown(self):
        '''
        Called on node shutdown.
        '''
        pass


if __name__ == '__main__':
    try:
        node = RLController()
        node.spin()
    except rospy.ROSInterruptException:
        pass
