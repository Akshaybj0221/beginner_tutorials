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
#include "beginner_tutorials/text.h"

std::string var = "Anything base - ";

bool add(beginner_tutorials::text::Request  &req,
         beginner_tutorials::text::Response &res)
{

  var = req.a;
  res.word = var;
//  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
//  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  ROS_INFO("Message changes!");
  return true;
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::ServiceServer service = n.advertiseService("add_text", add);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {

    std_msgs::String msg;

    std::stringstream ss;

 //   beginner_tutorials::AddTwoInts srv;   

 //   ros::ServiceServer service = n.advertiseService("add_two_ints", add);

    ROS_DEBUG_STREAM( "Counted to " << count );
    if (( count % 3) == 0) {
	ROS_INFO_STREAM(count << " is  divisible by 3.");
    }
    if (( count % 5) == 0  ) {
	 ROS_WARN_STREAM(count << " is  divisible by 5.");
    }
    if (( count % 10) == 0)  {
	ROS_ERROR_STREAM(count << " is  divisible by 10.");
    }
    if (( count % 20) == 0)  {
	ROS_FATAL_STREAM(count << " is  divisible by 20.");
    }
    
    ss << var << count;
   
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

