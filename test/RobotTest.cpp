/**
 * MIT License
 * Copyright (c) 2018 Indushekhar Singh
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.N CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

/**
* @file RobotTest.cpp
* @brief Robot class test
* @details Unit test implementation for Robot Camera class
* @author Indushekhar Singh
* @version 1.0
* @copyright MIT License (c) 2018 Indushekhar Singh
*/


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <Robot.hpp>



void callBack(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_DEBUG_STREAM("velocity.linear.x = " << msg->linear.x);
    ROS_DEBUG_STREAM("velocity.angular.z = " << msg->angular.z);
}

/*
* @brief Test for the velocity publsiher in Robot class
* @param  none
* @return none
*/


TEST(RobotTest, velocityPublisherTest) {
  // Create a node handle
  ros::NodeHandle nh;

  // Create a subscriber
  auto velocitySub_ = nh.subscribe("/mobile_base/commands/velocity", 1, callBack);
  // Waot for some time
  ros::WallDuration(1).sleep();
  // Spine in callbacks
  ros::spinOnce();
  // Check the number of publishers
  EXPECT_EQ(velocitySub_.getNumPublishers(), 1);
}

/*
* @brief Test for the laser scan subscriber in Robot class
* @param  none
* @return none
*/


TEST(RobotTest, laserScanSubscriberTest) {
  ros::NodeHandle nh;
  auto  pub = nh.advertise<sensor_msgs::LaserScan
      > ("/scan", 1);
  ros::WallDuration(1).sleep();
  EXPECT_EQ(pub.getNumSubscribers(), 1);
}

/*
* @brief Test for the camera subscriber in Robot class
* @param  none
* @return none
*/


TEST(RobotTest, cameraSubscriberTest) {
  ros::NodeHandle nh;
  auto  pub = nh.advertise<sensor_msgs::Image>
   ("/camera/rgb/image_raw", 1);
  ros::WallDuration(1).sleep();
  EXPECT_EQ(pub.getNumSubscribers(), 1);
}

