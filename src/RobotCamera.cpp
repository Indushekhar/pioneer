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
* @file RobotCamera.cpp
* @brief RobotCamera class
* @details Implementation of RobotCamera class
* @author Indushekhar Singh
* @version 1.0
* @copyright MIT License (c) 2018 Indushekhar Singh
*/

#include <ros/ros.h>
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "RobotCamera.hpp"


RobotCamera::RobotCamera() {
  diagnostic_ = true;
  captureImageFlag_ = false;
}

RobotCamera::~RobotCamera() {}

void  RobotCamera::robotCameraCallback(const sensor_msgs::Image msg) {
  // check if capture image flag is true
  if ( captureImageFlag_ == true ) {
    cv_bridge::CvImagePtr image_ptr;
    try {
      image_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR_STREAM("cv_bridge exception:" << e.what());
      return;
    }
    std::ostringstream filename;
    filename << "pioneerImage_" << ros::WallTime::now() << ".jpg";
    cv::imwrite(filename.str(), image_ptr->image);
    // save image
    ROS_INFO("Saving image %s to ~/.ros/", filename.str().c_str());
    captureImageFlag_ = false;
  }
}

bool RobotCamera::captureImage(pioneer::captureImageService::Request &req,
                 pioneer::captureImageService::Response &resp) {
  // store the service input to flag
  captureImageFlag_ = req.flag;
  // set the response to true
  resp.response = true;
  return resp.response;
}


bool RobotCamera::diagnosticTest() {
  return diagnostic_;
}
