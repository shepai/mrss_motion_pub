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
        self.localized={'/board0':False,'/board1':False,'/board2':False,'/board3':False,'/goal':False,'/obstacle0':False,'/obstacle1':False,'/obstacle2':False}
        self.angles={'/board0':0,'/board1':0,'/board2':0,'/board3':0,'/goal':0,'/obstacle0':0,'/obstacle1':0,'/obstacle2':0}
        self.all_local=0
        # END MRSS
        self.route_plan=None
        self.current_target=0
    def map_callback(self, msg):
        self.map = json.loads(msg.data)
        self.cmd = geometry_msgs.msg.Twist()
        if self.all_local<len(list(self.localized.keys())): #has not mapped out environment. 
            c=0
            key=""
            while c<len(list(self.localized.keys())) and self.localized.get(key,True)==False: #loop till found unknown
                key=self.localized(list(self.localized.keys())[c])
                c+=1
            GOAL1=self.map.get(key,[100,0]) #get current position
            if GOAL1[1]<0.05 and GOAL1[1]>-0.05: #facing the target
                self.all_local+=1 #increase counter so it checks next object
                self.cmd.angular.z=0
                self.localized[key]=GOAL1[0] #store x 
                #TODO Turn anglar velocities into actual angles
            elif GOAL1[1]<-0.05: #rotate to closest
                self.cmd.angular.z=-0.2
                self.angles[key]-=0.2
            elif GOAL1[1]>0.05: #rotate to closest
                self.cmd.angular.z=0.2
                self.angles[key]+=0.2
        else: #only run this if all mapping is done
            if self.route_plan==None: #route has not been plan
                #calculate each stop via A*
                import random
                self.route_plan=[random.choice(list(self.localized.keys()[0:4])),'/goal']
                self.current_target=0
            else: #follow route
                if self.current_target<len(self.route_plan):
                    TEMP_GOAL=self.map.get(self.route_plan[self.current_target],[100,0]) #get current position
                    if TEMP_GOAL[1]<0.1 and TEMP_GOAL[1]>-0.1: #facing the target
                        if TEMP_GOAL[0]>0.05: #if not near object go towards x
                            self.cmd.linear.x=0.2
                        else:
                            self.all_local+=1 #increase counter so it checks next object
                            self.cmd.angular.z=0
                            self.cmd.linear.x=0
                            self.localized[self.route_plan[self.current_target]]=TEMP_GOAL[0] #store x distance
                    elif TEMP_GOAL[1]<-0.1: #rotate to closest
                        self.cmd.angular.z=-0.3
                    elif TEMP_GOAL[1]>0.1: #rotate to closest
                        self.cmd.angular.z=0.3

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
