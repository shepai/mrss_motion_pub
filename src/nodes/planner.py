#!/home/sacha/go1-rl/go1-venv/bin/python
# Reads the map output (see map_broadcaster.py) and publishes twist commands to reach the goal

from curses import def_prog_mode
import numpy as np
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import json
import random
import math
import heapq

# A* algorithm implementation
def astar(start, goal, nodes):
    # Helper function to calculate the Euclidean distance between two points
    def distance(ar1,ar2):
        ar_1=100
        for i in range(len(nodes)): #gather node
            if nodes[i][0]==ar1 and nodes[i][1]==ar2:
                ar_1=nodes[i][2]
        return ar_1
    def get(a):
        ar_1=[]
        for i in range(len(nodes)): #gather node
            if nodes[i][0]==a:
                ar_1=nodes[i]
        return ar_1
    # Define the movement costs for horizontal, vertical, and diagonal movements
    move_costs = nodes

    # Initialize open and closed sets
    open_set = [(0, start)]  # Priority queue for nodes to be visited
    closed_set = set()       # Set of visited nodes
    came_from = {}           # Dictionary to store the previous node in the optimal path

    # Initialize g and f scores for the start node
    g_scores = {start: 0}
    f_scores = {start: distance(start, goal)}

    while open_set:
        # Get the node with the lowest f score
        current_f, current_node = heapq.heappop(open_set)

        if current_node == goal:
            # Reconstruct the path when the goal is reached
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start)
            path.reverse()
            return path

        closed_set.add(current_node)

        # Explore neighboring nodes
        for key in move_costs:
            current_node=get(current_node)
            if len(current_node)>0:
                move_cost=current_node[2]
                neighbor = current_node[1]
                tentative_g = g_scores[current_node[0]] + move_cost

                if neighbor in closed_set:
                    continue

                if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                    # Update the scores and add the neighbor to the open set
                    came_from[neighbor] = current_node[0]
                    g_scores[neighbor] = tentative_g
                    f_scores[neighbor] = tentative_g + distance(neighbor, goal)
                    heapq.heappush(open_set, (f_scores[neighbor], neighbor))

    # No path found
    return None



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
    def map_callback_SLAM(self, msg):
        self.map = json.loads(msg.data)
        self.cmd = geometry_msgs.msg.Twist()
        if self.all_local<len(list(self.localized.keys())): #has not mapped out environment. 
            c=0
            key=""
            while c<len(list(self.localized.keys())) and self.localized.get(key,True)!=False: #loop till found unknown
                key=list(self.localized.keys())[c]
                c+=1
            GOAL1=self.map.get(key,[100,0]) #get current position
            if self.angles[key][0]==0: #no angle has been set
                self.angles[key][1]=time.time() #start timer
            if GOAL1[1]<0.05 and GOAL1[1]>-0.05: #facing the target
                self.all_local+=1 #increase counter so it checks next object
                endSpeed=self.cmd.angular.z
                self.cmd.angular.z=0
                self.localized[key]=GOAL1[0] #store x 
                self.angles[key][1]=time.time()-self.angles[key][1] #get total time it took
                #Turn anglar velocities into actual angles
                angle = endSpeed * self.angles[key][1]
                self.angles[key][0]=angle
            elif GOAL1[1]<-0.05: #rotate to closest
                self.cmd.angular.z=-0.2
                self.angles[key][0]-=0.2
            elif GOAL1[1]>0.05: #rotate to closest
                self.cmd.angular.z=0.2
                self.angles[key][0]+=0.2
        else: #only run this if all mapping is done
            if self.route_plan==None: #route has not been plan
                #TODO calculate distaces from each point
                nodes=[]
                names=list(self.localized.keys())
                for i,key in enumerate(names):
                    if i!=0 and self.angles.get(names[i],None)!=None: #ignore first one
                        a=self.localized[names[i]]
                        b=self.localized[names[i-1]]
                        angle=math.radians(self.angles[names[i]][0]) #convert to radians
                        c=((a**2+b**2)-(2*a*b*math.cos(angle)))**0.5 #cosine rule
                        nodes.append([names[i],names[i-1],c])
                        nodes.append(['a',names[i],1])
                #TODO calculate each stop via A*
                #self.route_plan=[random.choice(list(self.localized.keys()[0:4])),'/goal']
                path = astar('a', '/goal', nodes)
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
                            self.cmd.linear.y=0
                    elif TEMP_GOAL[1]<-0.1: #rotate to closest
                        self.cmd.angular.z=-0.3
                    elif TEMP_GOAL[1]>0.1: #rotate to closest
                        self.cmd.angular.z=0.3
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
            self.cmd.linear.x=0.   
        elif GOAL1[1]>0.1:#If y distance is more than 10 cm turn the robot until it isn't
            self.cmd.angular.z=0.3
            self.cmd.linear.x=0. 
        elif GOAL1[0]< 0.: #If the goal is behind you, turn. 
            if GOAL1[1]<-0.:
                self.cmd.angular.z=-0.3 
                self.cmd.linear.x=0.     
            elif GOAL1[1]>0.:
                self.cmd.angular.z=0.3
                self.cmd.linear.x=0.
        else:
            if (-0.1<GOAL1[0]) and (GOAL1[0]<0.1): #if you are near the goal stop moving.
                self.cmd.linear.x=0.
                self.cmd.linear.y=0.
                self.cmd.angular.z=0.

            elif (self.map.get('/obstacle1',None)!=None) and (coords_1[0]>0) and (-0.3<coords_1[1]<0.3): #if you see an obstacle, move to the right or to the left, depending on where the farest board is from the obstacle
                idx_max_dist_y = np.argmax([abs(BOARD0[1]-coords_1[1]),abs(BOARD1[1]-coords_1[1]),abs(BOARD2[1]-coords_1[1]),abs(BOARD3[1]-coords_1[1])]) 
                if idx_max_dist_y == 0:
                    self.cmd.linear.y=0.2*(BOARD0[1]/abs(BOARD0[1]))
                    self.cmd.linear.x=0.
                elif idx_max_dist_y == 1:
                    self.cmd.linear.y=0.2*(BOARD1[1]/abs(BOARD1[1]))
                    self.cmd.linear.x=0.
                elif idx_max_dist_y == 2:
                    self.cmd.linear.y=0.2*(BOARD2[1]/abs(BOARD2[1]))
                    self.cmd.linear.x=0.
                else:
                    self.cmd.linear.y=0.2*(BOARD3[1]/abs(BOARD3[1]))
                    self.cmd.linear.x=0.
                           
            elif GOAL1[0]>0.1: #if the goal is in front of you , move forward
                self.cmd.linear.x=0.2
                self.cmd.linear.y=0.
                self.cmd.angular.z=0.
            else: #In case there are some cases i did not think about. 
                self.cmd.linear.x=0.
                self.cmd.linear.y=0.
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
