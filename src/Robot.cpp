
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
* @file Robot.cpp
* @brief Robot class
* @details Implementation of Robot class
* @author Indushekhar Singh
* @version 1.0
* @copyright MIT License (c) 2018 Indushekhar Singh
*/


#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "Robot.hpp"


Robot::Robot()  {
    // publish on topic /mobile_base/commands/velocity
    // to move the robot

    velocityPub_ = n_.advertise<geometry_msgs::Twist
      > ("/mobile_base/commands/velocity", 100);

    // subscribe to /scan topic to get laser scan data
    laserScanSub_ = n_.subscribe <sensor_msgs::LaserScan
      > ("/scan", 1000, &Exploration::checkObstacle,
                     &exploration_);
    // subscribe to /camera/rgb/image_raw topic
    // to get image data from the robot camera

    cameraSub_ = n_.subscribe <sensor_msgs::Image>
                   ("/camera/rgb/image_raw", 500,
                    &RobotCamera::robotCameraCallback, &robotCamera_);

    // publish the motionService to be called by client

    motionServer_ = n_.advertiseService("motionService",
                                          &Exploration::motion,
                                         &exploration_);
    // create timer instance of duration 45 seconds
    // which triggers the in place turn action

    startTurnTimer_ = n_.createTimer(ros::Duration(100),
                              &Exploration::startTurnTimer,
                        &exploration_);
    // create second timer of 48 seconds
    // which will be used to stop turn action

    stopTurnTimer_ = n_.createTimer(ros::Duration(102),
                              &Exploration::stopTurnTimer,
                        &exploration_);


    // publish the velocityChangeService to be called by client

    velocityChangeServer_ = n_.advertiseService("velocityChangeService",
                                          &Exploration::velocityChange,
                                         &exploration_);
    captureImageServer_ =  n_.advertiseService("captureImageService",
                      &RobotCamera::captureImage, &robotCamera_);
}

Robot::~Robot() {}

void Robot::run() {
    velocityPub_.publish(exploration_.explore());
}


