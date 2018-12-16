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


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
#include "Exploration.hpp"



Exploration::Exploration() {
  // Initialize robot velocities
  currentVelocity_.linear.x = 0.0;
  currentVelocity_.linear.y = 0.0;
  currentVelocity_.linear.z = 0.0;
  currentVelocity_.angular.x = 0.0;
  currentVelocity_.angular.y = 0.0;
  currentVelocity_.angular.z = 0.0;
  targetVelocity_.linear.x = 0.0;
  collisionFlag_ = false;
  stopMotion_ = false;
  turnFlag_ = false;
  velocityChangeFlag_ = false;
  diagnostic_ = true;
}

Exploration::~Exploration()  {}



void Exploration::startTurnTimer(const ros::TimerEvent& ) {
    turnFlag_ = true;
    ROS_INFO("Timer1 called");
}

void Exploration::stopTurnTimer(const ros::TimerEvent& ) {
    turnFlag_ = false;
    ROS_INFO("Timer2 called");
}

void Exploration::checkObstacle(const sensor_msgs::LaserScan msg) {
    // Iterate over range values
    collisionFlag_ = false;
    for (auto i : msg.ranges) {
     if ( i < 0.6 ) {
        collisionFlag_ = true;
      }
    }
}

bool Exploration::checkCollision() {
  return collisionFlag_;
}

geometry_msgs::Twist  Exploration::explore() {
    currentVelocity_.linear.x = 0.0;
    currentVelocity_.linear.y = 0.0;
    currentVelocity_.linear.z = 0.0;
    currentVelocity_.angular.x = 0.0;
    currentVelocity_.angular.y = 0.0;
    currentVelocity_.angular.z = 0.0;

    // start if stopMotion_ is true
    if ( stopMotion_ == false )  {
        if (collisionFlag_ && ~turnFlag_) {
            ROS_INFO("Obstacle Detected");
            std::random_device rand_;
            std::mt19937 rng(rand_());
            std::uniform_int_distribution<int> dis(15, 65);
            float randAngle = dis(rng);
            currentVelocity_.linear.x = 0.0;
           // currentVelocity_.angular.z = 0.25;
            currentVelocity_.angular.z = randAngle * (3.14 / 180);
        } else if ( turnFlag_ ) {
            ROS_INFO("making turn to explore");
            currentVelocity_.linear.x = 0.0;
            currentVelocity_.angular.z = 1.0;
        } else if ( velocityChangeFlag_ && ~collisionFlag_ ) {
            ROS_INFO("changing forward velocity");
            currentVelocity_.angular.z = 0.0;
            currentVelocity_.linear.x = targetVelocity_.linear.x;
        } else {
            ROS_INFO("moving forward");
            currentVelocity_.angular.z = 0.0;
            currentVelocity_.linear.x = 0.3;
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
  }
  return resp.response;
}

bool Exploration::velocityChange(pioneer::velocityChangeService::Request &req,
                   pioneer::velocityChangeService::Response &resp) {
    // store the input velocity value
    targetVelocity_.linear.x = req.velocity;
    // set the change flag to true
    velocityChangeFlag_ = true;
    // set response to  true
    resp.response = true;
    return resp.response;
}


bool Exploration::diagnosticTest() {
  return diagnostic_;
}

bool Exploration:: turnStatus() {
    return turnFlag_;
}
