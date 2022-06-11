# Custom ROS Message: Drive

## Package Information: robotROS ##
Firstly, the name of the package that was developed in this implementation is called robotROS. In the package, there are several folders: launch, msg, src, urdf, and world. In launch, there are two .launch file: one for launching only the environment and the other for launching both the robot and the environment. In msg, there is the the Drive msg created for this project. In src, there are various .py files for keyboard control and algorithm control. In urdf, there is a .sdf file for the robot. In world, there is a .sdf file for the environment and a .world file that saves the .sdf file in a .world format.

## Implementation Background ##
For demonstrating the knowledge and usage of ROS, Gazebo was utilized as the 3D physics engine and a custom robot was designed using the model editor built into Gazebo. The robot is a 1 m square box composed of two revolutionary joints for the two wheels (.3m radius) and 2 ball joints for the caster wheels.

![alt text](https://github.com/joshyeram/robotROS/blob/main/robotROS/Screen%20Shot%202022-06-11%20at%2015.19.39.png?raw=true)

The front of the robot is indicated by the green caster ball and the back is indicated by the red caster ball. The mass, inertia, and friction coefficients of the robot were remained unchanged from the default parameters assigned by Gazebo. The model was saved as a .sdf file and differ- ential drive controller plugin was added to control the revolutionary joints given the cmd vel topic published by the robotController node. A simple environment was also designed with different models and collision meshes to test the robot.

## New ROS msg for Robot Control ##
To control the robot, a new ROS msg was created to control the drive characteristic of the robot. In the Drive msg, there are two parameter: forward and turn. They are both float32 and can be between [2,-2] for forward and [-π, π] for turn. The forward parameter is the velocity and direction of the linear motion of the robot. The turn parameter is the speed and direction of the angular motion of the robot. The robot is able to traverse forward, reverse, and rotate around the z axis (normal to the ground plane).

## Nodes and Topics for Controlling Robot with Keyboard ##
There are 3 major nodes that are involved in controlling the robot in Gazebo with keyboard controls: driverKeyboard, robotControl, and gazebo. There are also two topics that were used to make this control possible: drive and cmd vel. In driver.py, it will use the teleop twist keyboard package used in class to read the current keyboard press and publish a Drive msg to the new /drive topic. The four keys it interprets are: w,a,s,d (w and s for forward and reverse, a and d for turning). For example, when w is pressed, a Drive msg with forward: 1 and turn: 0 is be published. When nothing is pressed, it will publish a Drive msg with 0 as forward and turn. The drive topic is updated at 10 hz.
![alt text](https://github.com/joshyeram/robotROS/blob/main/robotROS/message.png?raw=true)

## Launch File ##
In order to simplify the Gazebo launching process, a launch file was created to launch model and environment at once in gazebo. With this in mind, it will also run roscore as well. However, you may need to chmod +x nameOfFile.py if needed and run a catkin make to reconfigure the launch file.

## Using the Package ##
In order to use the this package, teleop twist keyboard needs to be installed for the keyboard control. This project would not have been possible without this package as there was a problem with using the keyboard library with python without having root permission with rosrun. The link to package is linked here: http://wiki.ros.org/teleop_twist_keyboard.

sudo apt-get install ros-noetic-teleop-twist-keyboard

You will need to run 3 terminal windows (or have 3 windows open on terminator) and initialize it.

source ~/catkin_ws/devel/setup.bash

After sourcing, this command will launch the gazebo environment with the robot.

roslaunch robotROS robotEnv1Driver.launch

After loading the environment, this will run the driverKeyboard node and wait until it has a sub- scriber.

rosrun robotROS driver.py

Finally, this will start the robotControl node and you will be able to move the robot in gazebo.

rosrun robotROS robotControl.py

Make sure you are in the driverKeyboard node where you ran driver.py as it only registers key within that window.

## Demonstration Video ##
Here is a quick demonstration video: https://youtu.be/sdrknm_vYFM
