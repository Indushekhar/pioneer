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


#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <pioneer/velocityChangeService.h>
#include <pioneer/motionService.h>


class Exploration {
 private:

  geometry_msgs::Twist targetVelocity_;
  geometry_msgs::Twist currentVelocity_;
  bool collisionFlag_;
  bool turnFlag_;
  bool stopMotion_;
  float threshold_;
  bool velocityChangeFlag_;
  bool diagnostic_;

 public:

  Exploration();
  ~Exploration();

  geometry_msgs::Twist explore();

  void checkObstacle(const sensor_msgs::LaserScan msg);

  bool checkCollision();

  bool velocityChange(pioneer::velocityChangeService::Request &req,
                   pioneer::velocityChangeService::Response &resp);

  bool motion(pioneer::motionService::Request &req,
                       pioneer::motionService::Response &resp);

  void startTurnTimer(const ros::TimerEvent& startTurn);

  void stopTurnTimer(const ros::TimerEvent& stopTurn);


  bool diagnosticTest();


};
