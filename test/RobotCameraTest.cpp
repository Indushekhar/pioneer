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
* @file RobotCameraTest.cpp
* @brief RobotCamera class test
* @details Unit test implementation for Robot Camera class
* @author Indushekhar Singh
* @version 1.0
* @copyright MIT License (c) 2018 Indushekhar Singh
*/


#include <ros/ros.h>
#include <gtest/gtest.h>
#include "../include/RobotCamera.hpp"



/*
* @brief Test for the diagnostic
* @param  none
* @return none
*/

TEST(RobotCamera, cameraDiagnosticTest) {
  RobotCamera robotCamObject_;
  EXPECT_EQ(robotCamObject_.diagnosticTest() , true);
}

/*
* @brief Test whether captureImageService is ready to call
* @param  none
* @return none
*/


TEST(RobotCamera, captureImageServiceTest) {
  // Create node handle
  ros::NodeHandle nh;
  RobotCamera robotCamObject_;

  ros::ServiceClient client =
      nh.serviceClient<pioneer::captureImageService>("captureImageService");
  // Check if the service exists
  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);
}

/*
* @brief Test captureImageService response
* @param  none
* @return none
*/


TEST(RobotCamera , capImgServiceResTest) {
  RobotCamera robotCamObject_;
  ros::NodeHandle nh;
  pioneer::captureImageService srv;
  srv.request.flag = true;
  EXPECT_TRUE(robotCamObject_.captureImage(srv.request, srv.response));
}

