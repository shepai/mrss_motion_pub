#import numpy as np
import random
import matplotlib.pyplot as plt
import numpy as np
import math

class Events:
    def __init__(self,boards=4,obs=3): #set up events
        self.events={'/goal':[0,0]}
        for i in range(boards):
            self.events['/board'+str(i)]=[0,0]
        for i in range(obs):
            self.events['/obstacles'+str(i)]=[0,0]
    def get_random(self,rangex=3,rangey=3):
        for key in self.events:
            self.events[key]=[random.randint(-rangex*100,rangex*100)/100,random.randint(-rangey*100,rangey*10)/100]
        return self.events

class Env:
    def __init__(self,rangex=3,rangey=3,b=0,o=0):
        self.e=Events(boards=b,obs=o)
        self.e.get_random()
        self.robot=[random.randint(-rangex*10,rangex*10)/10,random.randint(-rangey*10,rangey*10)/10] #robot position
        self.path=[]
        self.rangex=rangex
        self.rangey=rangey
        self.angle=math.pi/3
    def visualize(self): #show the simulation to user
        targets=self.e.events
        plt.scatter(self.robot[0],self.robot[1],label="Robot position")
        for key in targets:
            coord=targets[key]
            plt.scatter(coord[0],coord[1],label=key)
        if len(self.path)>0:
           ar=np.array(self.path)
           plt.plot(ar[:,0],ar[:,1]) 
        plt.legend(loc="upper right")
        plt.xlim([-5,5])
        plt.ylim([-5,5])
        plt.show()
    def reset(self): #randomize positions
        self.robot=[random.randint(-self.rangex,self.rangex),random.randint(-self.rangey,self.rangey)] #robot position
        self.e.get_random()
    def get_dists(self):
        e=self.e.events
        d={}
        for key in e:
            rx=self.robot[0]
            ry=self.robot[1]
            px=e[key][0]
            py=e[key][1]
            theta=self.angle
            delta_x=px-rx
            delta_y=py-ry
            angle=math.atan(delta_y/delta_x)
            x=math.cos(angle-theta) * math.sqrt(delta_x**2 + delta_y**2)
            y=math.sin(angle-theta) * math.sqrt(delta_x**2 + delta_y**2)
            if px<rx: x*=-1
            if py>ry: y*=-1
            print(rx,ry,"-->",px,py)
            print(x,y)
            print(round(rx+x,1),round(ry+y,1),"==",round(px,1),round(py,1) )
            if round(rx+x,1)==round(px,1) and round(ry+y,1)==round(py,1):
                d[key]=[x,y] #in sight
                plt.scatter(round(rx+x,1),round(ry+y,1),label="step")
            else:
                d[key]=[-1,-1] #not in sight
            
        return d
    def moveRobot(self,x,y,z):
        rx=self.robot[0]
        ry=self.robot[1]
        self.angle+=z
        #constrain rotation
        if self.angle>math.pi*2: self.angle=self.angle-2*math.pi 

        self.robot=[x+rx,y+ry]

env=Env()

    
#env.visualize()
for i in range(10):
    plt.cla()
    env.reset()
    print(env.get_dists()['/goal'])
    env.visualize()
    dist=env.get_dists()['/goal']
    
"""env.moveRobot(dist[0],dist[1],0)
print(env.get_dists()['/goal'])
env.visualize()"""
