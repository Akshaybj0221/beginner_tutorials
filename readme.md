#This is a ROS Publisher/Subscriber demo

NOTE: Its best to run the package in ROS kinetic and Ubuntu 16.04. If using some other version of ROS then it is must to make necessary changes in the "CMakeLists.txt" file of the "beginner_tutorials" package.

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

7. cpplint and cppcheck output files are added in the src folder.

8. cpplint folder contains 2 folders, 'Before' and 'After', which contains output files before fixing all the cpplint errors and after fixing all the cpplint errors respectively.

