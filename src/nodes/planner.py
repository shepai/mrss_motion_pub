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
        GOAL1=self.map['/goal']
        BOARD0=self.map['/board1']
        BOARD1=self.map['/board2']
        BOARD2=self.map['/board3']
        BOARD3=self.map['/board4']
        if self.map.get('/obstacle1',None)!=None:
            coords_1=self.map['/obstacle1']
        if self.map.get('/obstacle2',None)!=None:
            coords_2=self.map['/obstacle2']
        self.cmd = geometry_msgs.msg.Twist()
        if GOAL1[0]>0.1:
            if GOAL1[1]<-0.1:
                self.cmd.angular.z=-0.3
            elif GOAL1[1]>0.1:
                self.cmd.angular.z=0.3
            else:
                if (coords_1[0]>0) and (-0.1<coords_1[1]<0.1):
                    idx_max_dist_y = np.argmax([abs(BOARD0[1]),abs(BOARD1[1]),abs(BOARD2[1]),abs(BOARD3[1])]) 
                    if idx_max_dist_y == 0:
                        self.cmd.angular.y=0.2*(BOARD0[1]/abs(BOARD0[1]))
                    elif idx_max_dist_y == 1:
                        self.cmd.angular.y=0.2*(BOARD1[1]/abs(BOARD1[1]))
                    elif idx_max_dist_y == 2:
                        self.cmd.angular.y=0.2*(BOARD2[1]/abs(BOARD2[1]))
                    else:
                        self.cmd.angular.y=0.2*(BOARD3[1]/abs(BOARD3[1]))
                    
                
                else:
                    self.cmd.angular.x=0.2
        elif GOAL1[0]<0.1:
            if GOAL1[1]<0.:
                self.cmd.angular.z=-0.3
            else:
                self.cmd.angular.z=0.3
        else:
            self.cmd.angular.x=0.
            self.cmd.angular.y=0.
            self.cmd.angular.z=0.
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
