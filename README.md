# Flappy Bird Automation Game

This repository contains the Flappy Bird game modified to be controlled with ROS.

## Game Description

If Flappy Bird collides with the asteroids it would be fatal. Flappy has laser scanner that provides distance measurements. It will give you its velocity and the laserscans in return for an acceleration input. Flappy Bird only asks for 60 seconds of your guidance. Automate Flappy Bird go through as many asteroid lines as possible before the time runs out.  

![Flappy](flappy_cover.png)

## Getting Started
Install ROS and setup a workspace as covered here [[ROS install guide]](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).  
To add the automation game to your workspace clone the automation game repository to the source folder.

If you followed the ROS installation tutorial it will look like this,   
```
cd ~/catkin_ws/src/
```
Clone the automation game,
```
git clone https://github.com/JohsBL/flappy_automation_test.git
```
Make sure you have Pygame installed,
```
sudo apt-get install python-pygame
```

Then run the catkin_make command
```
cd ~/catkin_ws/
catkin_make
```

## Run The Game

To run the game we use the roslaunch command. There are two ROS launch files depending on if you want to run the game with C++ or python automation code (whatever you prefer).

For python open a terminal and run,
```
roslaunch flappy_automation_code flappy_automation_code_py.launch
```
For C++ open a terminal and run,
```
roslaunch flappy_automation_code flappy_automation_code_cpp.launch
```
A GUI will become visible with the game start screen. For now the automation code does not do anything other than printing out some laser ray information. To start the game and move the bird press the arrow keys. Tapping the up key &uarr; will start the game. You can then add velocity in a wanted direction by pressing the arrow keys &larr;&uarr;&darr;&rarr;. Notice that it is not possible to go backwards and if hitting an obstacle the game stops.

## Automate Flappy Bird
Now that we have gotten familiar with the game we want to control Flappy Bird. 

### Automation code
Python: **flappy_automation_code_node.py** in the **scripts** folder.  

C++: **flappy_automation_code.cpp** in the **src** folder and **flappy_automation_code.hpp** in the **include** folder.

### Build and run
Once the code has been modified run the catkin_make command again,
```
cd ~/catkin_ws/
catkin_make
```

Then launch the game as before.

For Python,
```
roslaunch flappy_automation_code flappy_automation_code_py.launch
```
For C++,
```
roslaunch flappy_automation_code flappy_automation_code_cpp.launch
```

## Other Information
Scaling: 1 pixel = 0.01 meter  
Game and sensor update rates: 30 fps   
The velocity measurement is noise free   
Max acceleration x: 3.0 m/s^2  
Max acceleration y: 35.0 m/s^2  
Axis convention: x &rarr;, y &uarr;  
[LaserScan message definition](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)

| Value        | Unit           | Topic  |
| ------------- |:-------------:| :-----:|
| Velocity      | m/s           | /flappy_vel |
| Acceleration  | m/s^2         | /flappy_acc |
| LaserScan     | Radians, meters      | /flappy_laser_scan |
