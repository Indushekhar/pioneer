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
* @file Exploration.cpp
* @brief Exploration class
* @details Implementation of Exploration class
* @author Indushekhar Singh
* @version 1.0
* @copyright MIT License (c) 2018 Indushekhar Singh
*/

#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdint.h>
#include <../include/Exploration.hpp>
#include <math.h>


Exploration::Exploration() { }

Exploration::~Exploration()  {}


void Exploration::startTurnTimer(const ros::TimerEvent& ) {

}

void Exploration::stopTurnTimer(const ros::TimerEvent& ) {

}

void Exploration::checkObstacle(const sensor_msgs::LaserScan msg) {
    // Iterate over range values

}

bool Exploration::checkCollision() {

  return true;
}

geometry_msgs::Twist  Exploration::explore() {
   //Initialize velocity
  // Avoid obstacle
  // Include behaviour from service calls


    return currentVelocity_;
}



bool Exploration::motion
    (pioneer::motionService::Request &req,
                       pioneer::motionService::Response &resp) {
  // Toggle pause motion flag:

  return true;
}

bool Exploration::velocityChange(pioneer::velocityChangeService::Request &req,
                   pioneer::velocityChangeService::Response &resp) {


    return true;

}


bool Exploration::diagnosticTest() {


  return true;
}
