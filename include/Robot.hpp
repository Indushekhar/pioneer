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
* @file Robot.hpp
* @brief Robot class
* @details Definition of Robot class
* @author Indushekhar Singh
* @version 1.0
* @copyright MIT License (c) 2018 Indushekhar Singh
*/

#ifndef INCLUDE_ROBOT_HPP_
#define INCLUDE_ROBOT_HPP_

#include <stdlib.h>
#include <ros/ros.h>
#include "Exploration.hpp"
#include "RobotCamera.hpp"


/**
 * @brief Robot class calls the object of Exploration and RobotCamera class
 *        and run the whole system.
 */

class Robot {
 public:
  /**
   * @brief Robot class constructor
   * @param  none
   * @return none
   */

  Robot();

   /**
   * @brief Robot class destructor
   * @param  none
   * @return none
   */

  ~Robot();

  /**
  * @brief : Method to run the turtlebot system for exploration
  * @param : None
  * @return : None
  */

  void run();


 private:
  /**
  * @brief RobotCamera Object
  */

  RobotCamera robotCamera_;

 /**
  * @brief Exploration object
  */

  Exploration exploration_;

 /**
 * @brief ROS Node handle
 */

  ros::NodeHandle n_;

  /**
  * @brief Velocity publisher
  */

  ros::Publisher velocityPub_;

  /**
   * @brief Image topic subscriber
   */
  ros::Subscriber cameraSub_;

  /**
  * @brief Laser scanner subsrciber
  */

  ros::Subscriber laserScanSub_;

  /**
  * @brief  Server for Image capture service
  */

  ros::ServiceServer captureImageServer_;

  /**
  * @brief Server for velocity change service
  */

  ros::ServiceServer velocityChangeServer_;

  /**
  * @brief Server for motion start stop service
  */

  ros::ServiceServer motionServer_;

  /**
  * @brief ROS timer for calling the start turn callback
  */

  ros::Timer startTurnTimer_;

  /**
  * @brief  Ros timer for calling the stop turn callback
  */

  ros::Timer stopTurnTimer_;
};


#endif  // INCLUDE_ROBOT_HPP_
