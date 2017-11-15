#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <tf/transform_listener.h>
#include <sstream>
#include "beginner_tutorials/text.h"

TEST(TestService, test_existence) {
   ros::NodeHandle nh;
   
   ros::ServiceClient client = nh.serviceClient<beginner_tutorials::text>("add_text");

   bool exists(client.waitForExistence(ros::Duration(10)));
   EXPECT_TRUE(exists);
}
