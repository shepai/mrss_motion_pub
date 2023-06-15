#!/usr/bin/env python3
# Reads the map output (see map_broadcaster.py) and publishes twist commands to reach the goal

from curses import def_prog_mode
import numpy as np
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import json
import numpy as np

def findGrid(events={'/board0':[0.2,0.5],'/board1':[0.1,0.3],'/board2':[-0.3,-0.2],'/board3':[0.1,-0.2],'/goal':[-0.3,-0.2],'/obstacle1':[-0.1,-0.2],'/obstacle1':[-0.3,-0.1]}):
    #get total distances from x and y to estimate grid size
    #convert to integers of single digit
    totalX=abs(int(events['/board0'][0]*10))+abs(int(events['/board2'][0]*10))
    totalY=abs(int(events['/board1'][1]*10))+abs(int(events['/board3'][1]*10))
    robot_x=int(events['/board0'][0]*10)
    robot_y=int(events['/board1'][1]*10)
    #devide into grid
    array=np.zeros((totalX,totalY))
    #calculate distances from target
    goal_x=abs(events['/goal'][0])*10
    goal_y=abs(events['/goal'][1])*10
    for x in range(array.shape[0]):
        for y in range(array.shape[1]):
            dist_from_target=(((goal_x-x)**2+(goal_y-y)**2)**0.5) /10 #euclidean distance divide by 10 to get in meters
            array[x][y]=dist_from_target
            #increase values that are on route to obstacles
            for target in [events.get('/obstacle0',None),events.get('/obstacle1',None),events.get('/obstacle2',None)]: #loop through 
                if target!=None: #if target exists
                    px=target[0]*10
                    py=target[1]*10
                    dist_from_target+=1/((((px-x)**2+(py-y)**2)**0.5) /10) #gather distance from target
                    array[x][y]+=dist_from_target
    new_vel=[0,0,0]
    min_v=100
    #loop through surrounding square
    for i in range(-1,2):
        for j in range(-1,2):
            if robot_x+i<totalX and robot_y+j<totalY and (i!=0 or j!=0): #check surrounding and not self
                if array[robot_x+i][robot_y+j]<min_v: #get smallest path
                    min_v=array[robot_x+i][robot_y+j]
                    new_vel=[i/10,j/10,0] #save direction as velocities
    return new_vel
    
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
        GOAL1 = self.map['/goal']
        BOARD0 = self.map['/board0']
        BOARD1 = self.map['/board1']
        BOARD2 = self.map['/board2']
        BOARD3 = self.map['/board3']
        if self.map.get('/obstacle1',None)!=None:
            coords_1=self.map['/obstacle1']
        if self.map.get('/obstacle2',None)!=None:
            coords_2=self.map['/obstacle2']
        self.cmd = geometry_msgs.msg.Twist()
        if GOAL1[1]<-0.1: #If y distance is more than 10 cm turn the robot until it isn't
            self.cmd.angular.z=-0.3      
        elif GOAL1[1]>0.1:#If y distance is more than 10 cm turn the robot until it isn't
            self.cmd.angular.z=0.3
        elif GOAL1[0]<-0.1: #If the goal is behind you, turn. 
            if GOAL1[1]<0.:
                self.cmd.angular.z=-0.3      
            elif GOAL1[1]>0.:
                self.cmd.angular.z=0.3
        else:
            if (-0.1<GOAL1[0]) and (GOAL1[0]<0.1): #if you are near the goal stop moving.
                self.cmd.linear.x=0.
                self.cmd.linear.y=0.
                self.cmd.angular.z=0.

            elif (self.map.get('/obstacle1',None)!=None) and (coords_1[0]>0) and (-0.1<coords_1[1]<0.1): #if you see an obstacle, move to the right or to the left, depending on where the farest board is (so the robot doesn't go out of bound)
                idx_max_dist_y = np.argmax([abs(BOARD0[1]),abs(BOARD1[1]),abs(BOARD2[1]),abs(BOARD3[1])]) 
                if idx_max_dist_y == 0:
                    self.cmd.linear.y=0.2*(BOARD0[1]/abs(BOARD0[1]))
                elif idx_max_dist_y == 1:
                    self.cmd.linear.y=0.2*(BOARD1[1]/abs(BOARD1[1]))
                elif idx_max_dist_y == 2:
                    self.cmd.linear.y=0.2*(BOARD2[1]/abs(BOARD2[1]))
                else:
                    self.cmd.linear.y=0.2*(BOARD3[1]/abs(BOARD3[1]))
                           
            elif GOAL1[0]>0.1: #if the obstacle is in front of you , move forward
                self.cmd.linear.x=0.2
            else: #In case there are some cases i did not think about. 
                self.cmd.linear.x=0.
                self.cmd.linear.y=0.
                self.cmd.angular.z=0.

        #experimental target code
        #vel=findGrid(events=self.map)
        #self.cmd.linear.x=vel[0]
        #self.cmd.linear.y=vel[1]
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
