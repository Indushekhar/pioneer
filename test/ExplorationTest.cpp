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
* @file ExplorationTest.cpp
* @brief Exploration class test
* @details Unit test implementation for Exploration class
* @author Indushekhar Singh
* @version 1.0
* @copyright MIT License (c) 2018 Indushekhar Singh
*/


#include <ros/ros.h>
#include <gtest/gtest.h>
#include "../include/Exploration.hpp"


/*
* @brief Test for the diagnostic
* @param  none
* @return none
*/

TEST(ExplorationTest_, diagnosticTest) {
  Exploration explorationTest_;
  EXPECT_EQ(explorationTest_.diagnosticTest() , true);
}

/*
* @brief Test whether motionService is ready to call
* @param  none
* @return none
*/

TEST(ExplorationTest_, motionServiceTest) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<pioneer::motionService>("motionService");
  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);
}

/*
* @brief verify motionService in Exploration class
*        Test whether robot is starting/stopping by service
* @param  none
* @return none
*/

TEST(ExplorationTest_ , motionServiceResponseTest) {
  Exploration explorationTest_;
  pioneer::motionService srv;
  srv.request.stop = true;
  EXPECT_TRUE(explorationTest_.motion(srv.request, srv.response));
  geometry_msgs::Twist velocity = explorationTest_.explore();
  EXPECT_EQ(velocity.linear.x, 0.0);
}


/*
* @brief Test whether velocityChangeService is ready to call
* @param  none
* @return none
*/

TEST(ExplorationTest_, velocityChangeServiceTest) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<pioneer::velocityChangeService>("velocityChangeService");
  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);
}

/*
* @brief verify veclocityChangeService response in Exploration class
* @param  none
* @return none
*/

TEST(ExplorationTest_ , velChangeServiceRespTest) {
  Exploration explorationTest_;
  pioneer::velocityChangeService srv;
  srv.request.velocity = 0.1;
  EXPECT_TRUE(explorationTest_.velocityChange(srv.request, srv.response));
  geometry_msgs::Twist velocity = explorationTest_.explore();
  EXPECT_EQ(velocity.linear.x, 0.1);
}

/*
* @brief Test if checkObstacle method is working correctly.
* @param  none
* @return none
*/

TEST(ExplorationTest_, collisionTest) {
  std::vector<float> range_array(100, 0.0);
  sensor_msgs::LaserScan msg;
  msg.angle_min = 0;
  msg.angle_max = 0;
  msg.angle_increment = 1;
  msg.time_increment = 0;
  msg.scan_time = 0;
  msg.range_min = 0;
  msg.range_max = 0;
  msg.ranges = range_array;
  msg.intensities.push_back(0);
  Exploration pioneerExplore_;
  pioneerExplore_.checkObstacle(msg);
  EXPECT_EQ(pioneerExplore_.checkCollision(), true);
  std::vector<float> range_array_(100, 4.0);
  msg.ranges = range_array_;
  pioneerExplore_.checkObstacle(msg);
  EXPECT_EQ(pioneerExplore_.checkCollision(), false);
}

/*
* @brief Test if explore method set correct velocity
*        when obstacle is near
* @param  none
* @return none
*/

TEST(ExplorationTest_, exploreTest) {
  Exploration explorationTest_;
  std::vector<float> range_array(100, 0.0);
  sensor_msgs::LaserScan msg;
  msg.angle_min = 0;
  msg.angle_max = 0;
  msg.angle_increment = 1;
  msg.time_increment = 0;
  msg.scan_time = 0;
  msg.range_min = 0;
  msg.range_max = 0;
  msg.ranges = range_array;
  msg.intensities.push_back(0);
  explorationTest_.checkObstacle(msg);
  geometry_msgs::Twist velocity = explorationTest_.explore();
  EXPECT_EQ(velocity.linear.x, 0.0);
}

/*
* @brief Test if explore method set correct velocity
*        when start turn timer is called.
* @param  none
* @return none
*/

TEST(ExplorationTest_ , timerTest) {
  Exploration explorationTest_;
  ros::Duration(100).sleep();
  geometry_msgs::Twist velocity = explorationTest_.explore();
  if (explorationTest_.turnStatus()) {
    EXPECT_EQ(velocity.linear.x, 0.0);
    EXPECT_EQ(velocity.angular.z, 1.0);
  }
}

