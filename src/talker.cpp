/************************************************************
 *                                                          *
 * Copyright (C) 2017 by Akshay Bajaj                       *
 *                                                          *
 ***********************************************************/

/**
 *   @file	listener.cpp
 *   @brief  	demo for ROS listener node
 *
 *   @author	Akshay Bajaj
 *   @date	10/30/2017
 */
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/AddTwoInts.h"

bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.word = "steing:";
  
//  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
//  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {

    std_msgs::String msg;

    std::stringstream ss;

    beginner_tutorials::AddTwoInts srv;   

    ss << "Custom made hello world " << count << "response " << srv.response.word;
   
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

