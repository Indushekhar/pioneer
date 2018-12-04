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


Exploration::Exploration() {
  currentVelocity_.linear.x = 0.0;
  currentVelocity_.linear.y = 0.0;
  currentVelocity_.linear.z = 0.0;
  currentVelocity_.angular.x = 0.0;
  currentVelocity_.angular.y = 0.0;
  currentVelocity_.angular.z = 0.0;
  targetVelocity_.linear.x = 0.0;
  collisionFlag_ = false;
  stopMotion_ = false;
  turnFlag_ = false ;
  velocityChangeFlag_= false;
  diagnostic_ = true;

}

Exploration::~Exploration()  {}



void Exploration::startTurnTimer(const ros::TimerEvent& ) {

    turnFlag_= true;
    ROS_INFO("Time1 called");
}

void Exploration::stopTurnTimer(const ros::TimerEvent& ) {

    turnFlag_= false;
    ROS_INFO("Timer2 called");
}

void Exploration::checkObstacle(const sensor_msgs::LaserScan msg) {
    // Iterate over range values
    collisionFlag_ = false;
    for (auto i : msg.ranges) {
     if ( i < 0.8) {
        collisionFlag_ = true;
      }
    }
}

bool Exploration::checkCollision() {

  return collisionFlag_;
}

geometry_msgs::Twist  Exploration::explore() {
   // ROS_INFO("calling explore method");
    currentVelocity_.linear.x = 0.00001;
    currentVelocity_.linear.y = 0.0;
    currentVelocity_.linear.z = 0.0;
    currentVelocity_.angular.x = 0.0;
    currentVelocity_.angular.y = 0.0;
    currentVelocity_.angular.z = 0.0000001;


    if (~stopMotion_)  {

        if (collisionFlag_) {
            ROS_INFO("Obstacle Detected");
            currentVelocity_.linear.x = 0.0;
            currentVelocity_.angular.z = 0.35;
           // currentVelocity_ = targetVelocity;

        }

        else if (turnFlag_ ) {
            ROS_INFO("making turn to explore");
            currentVelocity_.linear.x = 0.0;
            currentVelocity_.angular.z = 0.25;
            //currentVelocity_ = targetVelocity_;
           //ros::Duration(30).sleep();

        }

         else if ( velocityChangeFlag_)
         {
            ROS_INFO("changing forward velocity");
            currentVelocity_.angular.z = 0.0;
            currentVelocity_.linear.x = targetVelocity_.linear.x;
           // currentVelocity_ = targetVelocity_;
        }
        else {
            ROS_INFO("moving forward");
            currentVelocity_.angular.z = 0.0;
            currentVelocity_.linear.x = 0.3 ;
            //currentVelocity_ = targetVelocity_;
        }

    }

    return currentVelocity_;
}



bool Exploration::motion
    (pioneer::motionService::Request &req,
                       pioneer::motionService::Response &resp) {
  // Toggle pause motion flag:
  stopMotion_ = req.stop;
  resp.response = true;

  if (stopMotion_) {
    ROS_INFO_STREAM("Robot Stopped by the user");
  } else {
    ROS_INFO_STREAM("Starting the robot movement");
  }

  return resp.response;
}

bool Exploration::velocityChange(pioneer::velocityChangeService::Request &req,
                   pioneer::velocityChangeService::Response &resp) {

    targetVelocity_.linear.x = req.velocity;
    velocityChangeFlag_ = true ;
    resp.response = true;

    return resp.response;

}


bool Exploration::diagnosticTest() {
  return diagnostic_;
}
