#This is a ROS Publisher/Subscriber demo

To build and run the package follow the steps below:

1. Run roscore by typing $roscore on the terminal.

2. If you are using catkin, make sure you have sourced your workspace's setup.sh file after calling catkin_make. The steps to source setup file is as follows:

In your catkin workspace 
$ cd ~/catkin_ws 
$ source ./devel/setup.bash 

3. $ cd ~/catkin_ws/ 

4. $ catkin_make

5. Now, Run the publisher called "talker" by typing following command on the new terminal:
$ rosrun beginner_tutorials talker

6. Now, Run the publisher called "listener" by typing following command on the new terminal:
$ rosrun beginner_tutorials listener

