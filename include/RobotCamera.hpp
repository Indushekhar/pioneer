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
* @file RobotCamera.hpp
* @brief RobotCamera class
* @details Definition of Robot Camera class
* @author Indushekhar Singh
* @version 1.0
* @copyright MIT License (c) 2018 Indushekhar Singh
*/


#include <string>
#include <vector>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <pioneer/captureImageService.h>


class RobotCamera {
 private:

  bool captureImageFlag_;

  ros::ServiceClient robotCameraClient_;

  ros::NodeHandle nh_;

  bool diagnostic_;

 public:

  RobotCamera();
  ~RobotCamera();
  bool captureImage(pioneer::captureImageService::Request &req,
                 pioneer::captureImageService::Response &resp);

  void robotCameraCallback(const sensor_msgs::ImageConstPtr& msg);


  bool diagnosticTest();


};