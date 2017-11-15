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

TEST(testTf, tfWorking) {
   ros::NodeHandle nh1;

   tf::TransformListener tflistener;
   tf::StampedTransform transform;

   try {
      EXPECT_NO_FATAL_FAILURE(listener.waitForTransform("/world", "/talk", ros::Time(0), ros::Duration(0.5)));

      EXPECT_NO_FATAL_FAILURE(listener.lookupTransform("/world", "/talk", ros::Time(0), transform));
}
}

