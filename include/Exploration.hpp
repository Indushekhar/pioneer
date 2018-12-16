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
* @file Exploration.hpp
* @brief Exploration class
* @details Decleration of Exploration class
* @author Indushekhar Singh
* @version 1.0
* @copyright MIT License (c) 2018 Indushekhar Singh
*/

#ifndef INCLUDE_EXPLORATION_HPP_
#define INCLUDE_EXPLORATION_HPP_


#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <pioneer/velocityChangeService.h>
#include <pioneer/motionService.h>

/**
 * @brief Exploration class read the laser scanner data and
 *        move the robot in an unknown area with obstacle avoidance.
 *        Includes velocity change and motion service to  change velocity or
 *        start/stop the robot
 *
 */

class Exploration {
 public:
  /**
  * @brief  Constructor of Exploration class
  * @param  none
  * @return none
  */

  Exploration();

  /**
  * @brief  Destructor of Exploration class
  * @param  none
  * @return none
  */

  ~Exploration();

 /**
 * @brief Generate input velocity to run the robot
 * @param none
 * @return geometry_msgs::Twist input velocity at which robot will move
 */
  geometry_msgs::Twist explore();

 /**
 * @brief check the distance of obstacle from robot using laser scan
 * @param const sensor_msgs::LaserScan laser scan mmessage
 * @return none
 */

  void checkObstacle(const sensor_msgs::LaserScan msg);

  /**
 * @brief method to return the collision flag
 * @param none
 * @return bool collisionFLag_
 */

  bool checkCollision();

 /**
 * @brief callback back function for velocityChange service
 * @param &req is the request sent by the client
 * @param &resp is the response sent by the server
 * @return bool status of service call
 */

  bool velocityChange(pioneer::velocityChangeService::Request &req,
                   pioneer::velocityChangeService::Response &resp);

 /**
 * @brief callback back function for motion service
 * @param &req is the request sent by the client
 * @param &resp is the response sent by the server
 * @return bool status of service call
 */

  bool motion(pioneer::motionService::Request &req,
                       pioneer::motionService::Response &resp);

/**
 * @brief callback function for the startTurn timer
 * @param ros::TimerEvent  count the duration
 * @return none
 */

  void startTurnTimer(const ros::TimerEvent& startTurn);

 /**
 * @brief callback function for the stopTurn timer
 * @param ros::TimerEvent  count the duration
 * @return none
 */

  void stopTurnTimer(const ros::TimerEvent& stopTurn);

 /**
 * @brief  returns the turnFlag_
 * @param none
 * @return bool  Gives turnFlag_
 */

  bool turnStatus();

 /**
 * @brief  Gives the diagnostic
 * @param none
 * @return bool  Gives self diagnostic boolean variables
 */

  bool diagnosticTest();

 private:
 /**
 * @brief variable to store target velocity
 */
  geometry_msgs::Twist targetVelocity_;

 /**
 * @brief variable to store current velocity
 */
  geometry_msgs::Twist currentVelocity_;

 /**
 * @brief variable to store collision status
 */
  bool collisionFlag_;

 /**
 * @brief variable to store robot turn status
 */
  bool turnFlag_;

 /**
 * @brief variable to store robot start/motion status flag.
 */
  bool stopMotion_;

 /**
 * @brief variable to store robot veclocity change status
 */

  bool velocityChangeFlag_;

 /**
 * @brief variable for diagnostic
 */

  bool diagnostic_;
};

#endif  // INCLUDE_EXPLORATION_HPP_
