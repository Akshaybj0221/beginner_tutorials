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

## TF transform 
TF transforms are the way to define coordinate transforms between various frames. The talker node now publishes a non-zero transformation between `world` and `talk` frames. The easiest to check if the transforms are being broadcasted between the two frames is using `tf_echo`. To see the transforms between all the frames and how they are connected, one can use `view_frames` or `rqt_tf_tree`. 

Ensure that `talker_node` is already running. It can be run using the instructions given in the earlier sections. Now to see the transform being broadcasted, execute the following command:
```
<home>$ rosrun tf tf_echo /world /talk
``` 

Here, the first frame is the parent frame while the second frame is the child frame.  The output after executing the above command looks like below:
``` 
At time 1510737450.842
- Translation: [35.200, 35.200, 35.200]
- Rotation: in Quaternion [0.667, 0.667, 0.333, 0.009]
            in RPY (radian) [2.610, -0.446, 1.694]
            in RPY (degree) [149.556, -25.581, 97.070]
At time 1510737451.542
- Translation: [35.900, 35.900, 35.900]
- Rotation: in Quaternion [0.667, 0.667, 0.333, 0.009]
            in RPY (radian) [2.610, -0.447, 1.694]
            in RPY (degree) [149.570, -25.596, 97.071]
At time 1510737452.543
- Translation: [36.900, 36.900, 36.900]
- Rotation: in Quaternion [0.667, 0.667, 0.333, 0.009]
            in RPY (radian) [2.611, -0.447, 1.694]
            in RPY (degree) [149.588, -25.618, 97.072]

```

To generate a PDF containing the connections between reference frame to the other frame, type the command below:
```
<home>$ rosrun rqt_tf_tree rqt_tf_tree
```

Or use following,
```
<home>$ rosrun tf view_frames
```

## Testing using rostest/gtest 
Integration testing is very important to ensure that the newly created or modified modules does not break the running version of the code. This module has two unit tests which ensure that `add_text` service is running and that the transform being broadcasted is as expected. 

There are two ways to run the tests viz. using `catkin_make` and `rostest`. Before running the tests, ensure that `catkin_make` is invoked to build the units that are not being tested. The tests can be run using either of the following commands:

#### Using catkin_make 
```
<home>$ cd <path to directory>/catkin_ws
<workspace>$ catkin_make run_tests_beginner_tutorials
```

#### Using rostest
```
<home>$ rostest beginner_tutorials talkertest.launch
```

## Recording/Playing rosbag
`rosbag` allows to record all the data that is being published across all the nodes. Bag files are very useful when one wants to debug what is happening on the robot. However, rosbag does not record calls to services as well as responses of the services. It can be recorded from the commandline using `rosbag record -a`. And the recorded bag file can be played using `rosbag play <path to bag file>`. 

One can check the information of the bag file using `rosbag info <filename>` command. Sample output looks like below:
```
viki@ubuntu:~/catkin_ws/src/beginner_tutorials$ rosbag info results/recTalker.bag 
path:        results/recTalker.bag
version:     2.0
duration:    19.9s
start:       Nov 14 2017 23:11:43.20 (1510729903.20)
end:         Nov 14 2017 23:12:03.11 (1510729923.11)
size:        269.0 KB
messages:    1307
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      188 msgs    : std_msgs/String   
             /rosout       464 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   467 msgs    : rosgraph_msgs/Log 
             /tf           188 msgs    : tf2_msgs/TFMessage

```

`add_text` launch file of this package accepts a flag `record` that can be used to record the bag file which will record all the data that is being published on the `chatter` topic. It will save the file in `results` directory. This can be done using following commands:
```
<home>$ roslaunch beginner_tutorials add_text record:=true
```

Before proceeding to play the bag file, ensure that subscriber node is running as per the instructions given the earlier sections. To see the data stored in the bag file recorded using the above command:
```
<home>$ cd catkin_ws/src/beginner_tutorials/results
<results>$ rosbag play publisher.bag
```

This command will start playing the recorded bag file and the subscriber node which was running previously will be able to listen to the data that is being published. One can check the transform being broadcasted using the instructions given before. 

### Miscellaneous Instructions

1. cpplint and cppcheck output files are added in the *results* folder.

2. screenshots are added in the *output* folder.

3. *msg* folder is to be ignored as it has nothing relevant to the project.

4. *config* folder contains the custom log file.

