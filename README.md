# Pioneer
Pioneer : Exploratory Robot

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://travis-ci.org/Indushekhar/pioneer.svg?branch=master)](https://travis-ci.org/Indushekhar/pioneer)
[![Coverage Status](https://coveralls.io/repos/github/Indushekhar/pioneer/badge.svg?branch=master)](https://coveralls.io/github/Indushekhar/pioneer?branch=master)


## Project Overview

The objective of this project is to design a software stack which will enable the Turtlebot2 robot
to explore an unknown environment autonomously. After exploration, it will also generate an
occupancy grid map. The robot will demonstrate robust obstacle avoidance with some added
feature such as visual feedback from onboard camera, velocity control, Image capture option and
start/stop control. All the code though will be tested on Gazebo simulations.

This product can be very useful in the exploration of a disaster area and provide aid in rescue
effort by locating victims, delivering medical aids etc. The addition features will be also very
helpful in these operations. For example, the user can slow down the robot if he found something
useful (say injured human being) from the camera feedback and then deliver the aid. Other
application could be hazardous area (say radioactive radiation, poisonous gas exposed area etc)
exploration where human access has very high risk involved. It can also be used for generating
the occupancy map which can be used for indoor navigation purposes.


 ![Test Image 1](https://github.com/Indushekhar/pioneer/blob/master/results/GazeboWorldV1.png) 


## About the Developers

I am Indushekhar Singh. I am currently pursing Masters in Robotics at University of Maryland - College Park. I hold a Bachelors degree in Electrial & Electronics Engineering from, NIT Jamshedpur India. Along with that I brings 3 years of work experience in the area of Industrial Automation. I am also comfortable working in MATLAB and Python. I have done few projects in these two languages too.


## Presentation 

Presentation slides can be found here : 
[link](https://docs.google.com/presentation/d/1i1OtTYiq-vU1j2jFT9TYRwO_zqju3rAUdaMAnnEWUF8/edit#slide=id.p)

Presentation video can be seen here :
[video link](https://www.youtube.com/watch?v=RE3-FI6Z5AU&feature=youtu.be)

## Dependencies

To run the program you need to have the following dependencies on your system:

1. Ubuntu (Xenial) 

2. ROS Kinetic Kame

3. Gazebo 7.x (part of ros-kinetic-desktop-full package)

4. Turtlebot simulation package

5. Gmapping

6. map_server

7. image_view


To install ROS, follow the instructions on this [link](http://wiki.ros.org/kinetic/Installation)

Don't forget to setup the ROS environment by adding the following line in your .bashrc :

```
$ source /opt/ros/kinetic/setup.bash

```

To install the turtlebot packages, run the following after installing ROS Kinetic on your ubuntu 16.04.

```
sudo apt-get install ros-kinetic-turtlebot-*

```

To install gmapping, In a terminal run :

```
sudo apt-get install ros-kinetic-slam-gmapping

```

To install map_server, In a terminal run:

```
sudo apt-get install ros-kinetic-map-server

```

To install image_view, run :

```
sudo apt-get install ros-kinetic-image-view

```


## Build Instructions 

Following the environment setup, next step is to create the workspace.

```
$ cd <path where workspace needs to be created>
$ mkdir -p <name of workspace>
$ cd <workspace>
<workspace>$ catkin_make

```

Now, for building the main branch

```
<home>$ cd <workspace>/src
<workspace>/src$ git clone --recursive https://github.com/Indushekhar/pioneer
<workspace>/src$ cd ..
<workspace>$ catkin_make 

```

Just like the path of the ROS was sourced in .bashrc file, same needs to be done for the workspace by writing 

```
source <path to workspace>/devel/setup.bash
```
in the .bashrc. This will avoid the needs of sourcing everytime we run the package.


## Running the System demo 

To run the system demo please follow the instruction given in this section. Open a terminal and launch a file using given below:

```
$ roslaunch pioneer pioneer.launch

```
The command given above will spwan the robot in a custom gazebo world and launch the rviz plugin. It will also start gmapping node which will be used to map the environment.

Now robot will start moving the gazebo simulation.

### Running the demo with image_view

If you want the see the camera feedback in real time, run the following launch file instead :

```
$ roslaunch pioneer pioneer_image_view.launch

```
This is will open a image_view window where you can see the real time camera feedback.

## Saving the map

You can see the map created in real time in rviz window. Once the exploration is complete, run the following command to save the map :

```
<path where map is to be saved> $ rosrun map_server map_saver -f <map_name>

```



## Services for the user

Along with the core exploration task, the system is capable of supporting additional user demands if required.

### Image Capture Service

The user any time during the simulation can capture an image and save on their disk using captureImageService. To run the service run following command on a new
terminal :

```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosservice call /captureImageService true

```
The files will get saved in ./ros folder. To view the files. In a new terminal:

```
gnome-open ~/.ros

```
If you do not have gnome-open installed. In a terminal run :

```
sudo apt install libgnome2-bin

```


### Veclocity Change Service

User can any time incresese/decrease the linear velocity of the robot. To increase or decrease the velocity run following command on a new
terminal :

```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosservice call /velocityChangeService <value>

```
for example , setting the velocity to 0.1 m/s, run :

```
rosservice call /velocityChangeService 0.1

```

### Start/Stop Service

User can any time start/stop the robot in simulation using motionService. To stop the robot if already moving, run following command on a new
terminal :

```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosservice call /motionService true

```

If the robot is in stop position, to move the robot ,  run following command on a new
terminal :

```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosservice call /motionService false

```

## Testing 

The unit tests for this package can be ran using the following commands. After cloning the repository, navigate to the root of the workspace and run the following command:

```
$ catkin_make run_tests 

```

It can also be run by using launch file :

```
$ rostest pioneer pioneer_test.launch

```

## Recording bag files

The data publsihed during the simulation can be recoreded in a bag file using the launch file by running the following command :

```
<home>$ roslaunch pioneer pioneer_image_view.launch record:=true

```

This will record data for 45 seconds of all the topic except /camera/* topics and save a bag file in the results folder of the package.


## Playing back bag file

To inspect the bag file, In a new terminal: 

```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ cd src/pioneer/results
$ rosbag info pioneer.bag

```

## Solo Iterative Process and Sprint Planning

While developing this module SIP process was followed. The link to the SIP sheet for this module is here.

https://docs.google.com/spreadsheets/d/1h1utyFEapiUcafLp1L-bTxqeRQMaXjP8P0F8vSe2olY/edit?usp=sharing


Sprint planning notes can be found here.

https://docs.google.com/document/d/1AY8NstnOtWDZ6wul_0_c_aTAbeUBaYTrImGRCwSZni4/edit?usp=sharing


## License

MIT License

Copyright (c) 2018 Indushekhar Singh

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
