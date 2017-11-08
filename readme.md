# Introduction to ROS: Beginner Tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/Akshaybj0221/beginner_tutorials/blob/WEEK10_HW/LICENSE)

This repository showcases the tutorials on ROS and is a part of ENPM808X course at University of Maryland.

## Dependencies 
The dependencies of this repository are:

```
* Ubuntu 16.04
* ROS Kinetic Kame
```

Before proceedigng to install ROS, ensure that version of Ubuntu is 16.04. To install ROS follow the steps given below:

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
```

After installation of ROS, the next step is to initialize and install its dependencies using following commands:

```
$ rosdep init
$ rosdep update 
```

The next step is to setup ROS environment which can be done using following commands:

```
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Following the environment setup, next step is to create the workspace.

```
$ cd <path where workspace needs to be created>
$ mkdir -p <name of workspace>
$ cd <workspace>
<workspace>$ catkin_make
```

Just like the path of the ROS was sourced in *.bashrc* file, same needs to be done for the workspace by writing `source <path to workspace>/devel/setup.bash` at the end of the *.bashrc* file.
This avoids the need to source every time one needs to use workspace's packages.

## Building the code

The code can be built by cloning the repository and executing following steps:
```
<home>$ cd <workspace>/src
<workspace>/src$ git clone https://github.com/Akshaybj0221/beginner_tutorials.git
<workspace>/src$ cd ..
<workspace>$ catkin_make 
```

## Running the code

This following tutorial deals with *services* and *launch* files. Launch files are very useful in case where multiple nodes needs to be started from within the package. One can also set the arguments of the nodes from within the launch files. The file *talk.launch* does that and is capable of running two nodes at one time which are 'talker' and 'listener'.

*Services* are also a very useful tool of ROS. While publisher/subscriber communication model is very useful for continuous communication, server/client communication has its own uses. Server/client model is very useful when one has to execute certain process only if the need arises by requesting the *service*. This code has a service called *add_text* which can be used whenever one wants to change the message being published. It takes an input the string which needs to be published and responses with a *bool* value.

### Running using *rosrun* commands
Please ensure that *rosmaster* is running before executing following steps. *rosmaster* can be started by following command.
```
<home>$ roscore
```

To start the *talker* node follow the steps given below:
```
<home>$ rosrun beginner_tutorials talker
```

*listener* node can be started by following commands:
```
<home>$ rosrun beginner_tutorials listener
```

Starting the nodes using the steps above, prints out `Base String <count_value>` on the console. 

### Running using *roslaunch* 
To start both the nodes with a single command, *launch* file can be created and used to launch both the nodes. To *launch* both nodes execute following command:
```
<home>$ roslaunch beginner_tutorials talk.launch 
```

Please note here that it is not mandatory to start *rosmaster* node while using *launch* file. It starts when the file is launched if it is not running.

### Calling the service 
Service can be called from the *terminal* when both the nodes are running. This is necessary because *talker* node is the server for service while *listener* node is client of the service. Thus, in order for the service to execute properly both server and client should run properly.
```
<home>$ rosservice call /add_text <string to be published>
```

Consider for example one wants to change the message being published to `Akshay <count value>` This can be done as follows:
```
<home>$ rosservice call /add_text Akshay
```

### Launching logger GUI
Logger GUI can be used to check the logged messages. To launch logger GUI, run the following lines:
```
<home>$ rosrun rqt_console rqt_console
```


### Miscellaneous Instructions

1. cpplint and cppcheck output files are added in the *results* folder.

2. screenshots are added in the *output* folder.

3. *msg* folder is to be ignored as it has nothing relevant to the project.

4. *config* folder contains the custom log file.

