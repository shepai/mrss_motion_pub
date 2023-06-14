#  MRSS Go1 Motion Package
This repository contains the skeleton to write high level motion planning code for the 2023 Montreal Robotics Summer School (MRSS).

This code should be used as a package in a [TagSLAM MRSS](https://github.com/sachaMorin/tagslam_root/blob/master/README-MRSS.md) workspace. For MRSS, we will
provide desktops where TagSLAM is already installed. You can also install the TagSLAM MRSS stuff on your own machine.

## Installation Instructions
First make sure to fork this repository and **RENAME IT** to some ```$TEAM_NAME```.

Now clone your package in tagslam_root:
```shell
cd ~/tagslam_root/src
git clone $YOUR_FORK_URL
```

Your first commit should be to replace all instances of ```mrss_motion``` with ```$TEAM_NAME``` in the following files:
- ```tagslam_root/src/$TEAM_NAME/package.xml```
 - ```tagslam_root/src/$TEAM_NAME/CMakeLists.txt```
 - ```tagslam_root/src/$TEAM_NAME/src/launch/motion_planning.launch```

Once this is done, build your package

```shell
cd ~/tagslam_root
catkin_make
```
## Usage
In any terminal where you need to use your motion planning package, you should run
```shell
source ~/tagslam_root/devel/setup.bash
```

Now make sure that the main [TagSLAM MRSS launch file](https://github.com/sachaMorin/tagslam_root/blob/master/README-MRSS.md) is running in another terminal:

```shell
roslaunch tagslam mrss_laptop.launch rviz:=1
```

Then you can launch motion planning with
```shell
roslaunch $TEAM_NAME motion_planning.launch twist:=0 rl_policy:=0
```
Setting ```twist:=1``` will launch the Unitree controller. The controller will listen to the ```cmd_vel``` topic and track
the velocities published by the planner node.

Setting ```rl_policy:=1``` will launch an RL controller. Make sure to update your policy path in ```tagslam_root/src/$TEAM_NAME/nodes/rl_policy.py```.

**Do not use both twist and rl_policy at the same time.**

## Workshop
The launch file spins up 3 ROS nodes:
- The **map_broadcaster** node. **FIX ME!**
- The **planner** node. **FIX ME!**
- The **goal_broadcaster** node.

At first, we will aim to:
- Finish the implementation of the **map_broadcaster** node.
- Add basic goal reaching capabilities to the **planner** node.

Following the appropriate lecture, we will attempt to add obstacle avoidance capabilities to the **planner** node following a lecture on planning and motion planning.

All relevant source code is under ```tagslam_root/src/$TEAM_NAME/src/nodes```.


## map_broadcaster node
 TagSLAM estimates and publishes all sorts of transforms to the ```tf``` topic. The **map_broadcaster** node should read relevant transforms (e.g., the goal, obstacles) and save them to a Python Dict. 
 
The Dict is then
serialized and streamed to a UDP port and to the ```/map``` ROS topic. RL stuff will likely use the UDP port while
the **planner** will rely on the ```/map``` topic.

The **map_broadcaster** currently publishes a single dummy transform. Add more transforms! You may want to have a look at the [tf documentation](http://wiki.ros.org/tf#:~:text=tf%20is%20a%20package%20that,any%20desired%20point%20in%20time.).

Here are some useful commands. This one allows to visualize the transform tree and undertand the objects you can query. 
You can safely igonore the ```camera_link``` and ```uodom``` trees, as well as the ```trunk``` subtree:
```shell
rosrun rqt_tf_tree rqt_tf_tree 
```
You want to print out the ```/map```  topic
```shell
rostopic echo /map
```



## planner node

This node should read the map from the ```/map``` topic, run some planning computations and publish twist commands (linear and angular velocities) to the ```/cmd_vel``` topic.

You should begin by understanding how to move the robot around. You may try 
- Moving the robot forward;
- Having the robot rotate in place;
- Having the robot walk in a circle.

Following this, you should start using the ```map``` to move the robot the the ```goal``` pose.


**Please always run low velocities when first testing your code (e.g, 0.15). The robot does not need to sprint!**

If you need to visualize your twist commmands, this command may be useful
```shell
rostopic echo /cmd_vel
```

## goal_broadcaster node
Currently, the goal position can be set in front of any one of the "boards" (the tiles or posters with 4 static april tags).
By default, the goal will be `board1`. A parameter controls the goal object and can be changed with

```shell
rosparam set /mrss_motion/goal_object board0
rosparam set /mrss_motion/goal_object board1
rosparam set /mrss_motion/goal_object board2
rosparam set /mrss_motion/goal_object board3
```
This node should work out of the box.

## ROS Bag
If you have ROS on your computer, the main [TagSLAM MRSS](https://github.com/sachaMorin/tagslam_root/blob/master/README-MRSS.md) README contains
instructions to playback all topics. This will allow you to work on your code even if you do not currently have 
access to the robot.