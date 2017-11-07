/************************************************************
 *                                                          *
 * Copyright (C) 2017 by Akshay Bajaj                       *
 *                                                          *
 ***********************************************************/

/**
 *   @file	  listener.cpp
 *   @author	  Akshay Bajaj
 *   @copyright   MIT License
 *
 *   @brief  	  Demo for ROS listener node   
 *   @date	  10/30/2017
 *   @updateDate  11/06/2017
 *
 *   @section	  DESCRIPTION
 *
 *   This program is a part of the beginner tutorials in ROS
 *   It defines the subscriber (listener node)
 */


#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "listener");


  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);


  ros::spin();

  return 0;
}

