/************************************************************
 *                                                          *
 * Copyright (C) 2017 by Akshay Bajaj                       *
 *                                                          *
 ***********************************************************/

/**
 *   @file	  talker.cpp
 *   @author	  Akshay Bajaj
 *   @copyright   MIT License
 *
 *   @brief  	  Demo for ROS talker node   
 *   @date	  10/30/2017
 *   @updateDate  11/06/2017
 *
 *   @section	  DESCRIPTION
 *
 *   This program is a part of the beginner tutorials in ROS which defines the publisher (talker node)
 */

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/text.h"


// Initializing base string
std::string var = "Base String ";

bool add(beginner_tutorials::text::Request  &req,
         beginner_tutorials::text::Response &res)
{

  var = req.a;
  res.word = var;
  // Print message notifying change in the string on service call
  ROS_INFO("String changes!");
  return true;
}


int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  // setting default frequency to 10
  int freq = 10;

  // Checking number of arguments
  if(argc == 2) {
        // Changing the frequency to the argument value
	ROS_DEBUG_STREAM("Argument is" << argc[1]);
	freq = atoi(argv[1]);
  }

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::ServiceServer service = n.advertiseService("add_text", add);

  ros::Rate loop_rate(freq);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;

 //   beginner_tutorials::AddTwoInts srv;   

 //   ros::ServiceServer service = n.advertiseService("add_two_ints", add);

    /**
     * Adding a logic to switch the message logging from Info, Warn, error, and fatal.
     */
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

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

