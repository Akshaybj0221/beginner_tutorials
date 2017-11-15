/**********************************************************************************
 *                                                          			  *
 * Copyright (C) 2017 by Akshay Bajaj                       			  *
 *										  *
 * Permission is hereby granted, free of charge, to any person obtaining a copy   *
 * of this software and associated documentation files (the "Software"), to deal  *
 * in the Software without restriction, including without limitation the rights   *
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell	  *
 * copies of the Software, and to permit persons to whom the Software is	  *
 * furnished to do so, subject to the following conditions:			  *
 *										  *
 * The above copyright notice and this permission notice shall be included in	  *
 * all copies or substantial portions of the Software.				  *
 *										  *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR	  *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,	  *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE	  *
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER	  *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  *
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  *
 * SOFTWARE.                                                         		  *
 *********************************************************************************/

/**
 *   @file	  talkerTest.cpp
 *   @author	  Akshay Bajaj
 *   @copyright   MIT License
 *
 *   @brief  	  Demo for ROS talker node   
 *   @date	  11/14/2017
 *
 *   @section	  DESCRIPTION
 *
 *   This program is the testing code for the package beginner_tutorials of the beginner tutorials series in ROS */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <tf/transform_listener.h>
#include <sstream>
#include "beginner_tutorials/text.h"




/**
 * @brief      To test the service
 */

TEST(TestService, test_existence) {
  ros::NodeHandle nh;
  ros::ServiceClient
client = nh.serviceClient<beginner_tutorials::text>("add_text");

  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);
}


/**
 * @brief      To test talker tf broadcaster
 */


TEST(testTf, tfWorking) {
  ros::NodeHandle nh1;

  tf::TransformListener tflistener;
  tf::StampedTransform transform;

  try {
  EXPECT_NO_FATAL_FAILURE(tflistener.waitForTransform
("/world", "/talk", ros::Time(0), ros::Duration(0.5)));

  EXPECT_NO_FATAL_FAILURE(tflistener.lookupTransform
("/world", "/talk", ros::Time(0), transform));
} catch (tf::TransformException ex) {
      ADD_FAILURE();
  }
}

