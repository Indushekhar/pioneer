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


#ifndef INCLUDE_ROBOTCAMERA_HPP_
#define INCLUDE_ROBOTCAMERA_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <pioneer/captureImageService.h>
#include <string>
#include <vector>



/**
 * @brief RobotCamera class capture image using captureImage service
 */


class RobotCamera {
 public:
  /**
  * @brief  Constructor of RobotCamera class
  * @param  none
  * @return none
  */
  RobotCamera();

  /**
  * @brief  Destructor of RobotCamera class
  * @param  none
  * @return none
  */

  ~RobotCamera();

 /**
 * @brief callback back function for captureImage service
 * @param &req is the request sent by the client
 * @param &resp is the response sent by the server
 * @return bool status of service call
 */

  bool captureImage(pioneer::captureImageService::Request &req,
                 pioneer::captureImageService::Response &resp);

 /**
 * @brief callback function for the camera subscriber
 *        saves an image locally if take image flag is true
 * @param const sensor_msgs::ImageConstPtr  A constant image pointer of what robot is seeing
 * @return none
 */

  void  robotCameraCallback(const sensor_msgs::Image msg);

 /**
 * @brief  Gives the diagnostic
 * @param none
 * @return bool  Gives self diagnostic boolean variables
 */

  bool diagnosticTest();


 private:
 /**
 * @brief flag for capturing the image
 */
  bool captureImageFlag_;

  /**
  * @brief registers client for camera service
  */


 /**
 * @brief Ros node handle
 */

  ros::NodeHandle nh_;


 /**
 * @brief variable for diagnostic
 */

  bool diagnostic_;
};

#endif  // INCLUDE_ROBOTCAMERA_HPP_
