#!/usr/bin/env python3
# Reads the map output (see map_broadcaster.py) and publishes twist commands to reach the goal

import numpy as np
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import json


class Planner:
    def __init__(self):
        super().__init__()

        # Initialize Node
        rospy.init_node('Planner', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # Initialize subscriber
        self.map_sub = rospy.Subscriber('/map', String, self.map_callback)
        self.map = None
        
        # Intialize publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.cmd = None
        self.rate = rospy.Rate(10)  # Publisher frequency

        # TODO BEGIN MRSS: Add attributes (If needed)

        # END MRSS

    def map_callback(self, msg):
        self.map = json.loads(msg.data)

        # TODO BEGIN MRSS: Use map for planning
        GOAL1=self.map['/goal'] #gives x and y
        #GOAL2=self.map['/goal2'] # assuming goal 2 is the side gives x and y
        #get angle between
        #angle=math.atan(GOAL1[1]/GOAL1[0])
        #s=d/t
        #time=10
        #self.cmd.linear.x=GOAL1[0]/time
        #self.cmd.linear.y=GOAL1[1]/time
        
        # END MRSS

        # Twist
        self.cmd = geometry_msgs.msg.Twist()
        ang_speed=0
        if GOAL1[1]<-0.1:
            self.cmd.angular.z=0.15
        elif GOAL1[1]>0.1:
            self.cmd.angular.z=-0.15
        elif GOAL1[0]>0.1:
            self.cmd.linear.x = 0.1
        else:
            self.cmd.linear.x = 0.
            self.cmd.linear.y = 0.
            self.cmd.angular.z=0
        
        # END MRSS

    def spin(self):
        '''
        Spins the node.
        '''
        try:
            while not rospy.is_shutdown():
                if self.cmd is not None:
                    # Publish
                    self.cmd_pub.publish(self.cmd)
                else:
                    rospy.logwarn("SKIP")

                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down planner.")

    def on_shutdown(self):
        '''
        Called on node shutdown.
        '''
        pass


if __name__ == '__main__':
    try:
        node = Planner()
        node.spin()
    except rospy.ROSInterruptException:
        pass
