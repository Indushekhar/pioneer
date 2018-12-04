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
#include <Exploration.hpp>

/*
* @brief Test for the diagnostic
*/

TEST(TESTSuite, diagnosticTest) {
  Exploration explorationTest_;
  EXPECT_EQ(explorationTest_.diagnosticTest() , true);
}

/*
* @brief Test whether motionService is ready to call
*/

TEST(TESTSuite, motionServiceTest) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<pioneer::motionService>("motionService");
  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);
}


/*
* @brief Test whether velocityChangeService is ready to call
*/

TEST(TESTSuite, velocityChangeServiceTest) {
  ros::NodeHandle nh;

  ros::ServiceClient client =
      nh.serviceClient<pioneer::velocityChangeService>("velocityChangeService");
  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);
}


/*
* @brief Test if checkObstacle method is working correctly.
*/

TEST(TESTSuite, collisionTest) {

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

  EXPECT_EQ( pioneerExplore_.checkCollision(), true);

  std::vector<float> range_array_(100, 4.0);

  msg.ranges = range_array_;

  pioneerExplore_.checkObstacle(msg);

  EXPECT_EQ(pioneerExplore_.checkCollision(), false);

}



int main(int argc, char** argv) {
  ros::init(argc, argv, "pioneerTest");

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

