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

#include <stdlib.h>
#include <ros/ros.h>
#include <Exploration.hpp>
#include <RobotCamera.hpp>


class Robot {


  private:

 // RobotCamera Object

  RobotCamera robotCamera_;

 // Exploration object

  Exploration exploration_;

 // ROS Node handle

  ros::NodeHandle n_;

  // Velocity publisher

  ros::Publisher velocityPub_;

 // Image topic subscriber

  ros::Subscriber cameraSub_;

  // Laser scanner subsrciber

  ros::Subscriber laserScanSub_;

  // Server for Image capture service

  ros::ServiceServer captureImageServer_;

  // Server for velocity change service

  ros::ServiceServer velocityChangeServer_;

  // Server for motion start stop service

  ros::ServiceServer motionServer_;

  // ROS timer for calling the start turn callback

  ros::Timer startTurnTimer_;

  // Ros timer for calling the stop turn callback

  ros::Timer stopTurnTimer_;



 public:
  /**
   * @brief Robot constructor
   */
  Robot();

   /**
   * @brief Robot destructor
   */

  ~Robot();

  /**
  * @brief : method to run the turtlebot system for exploration
  * @param : None
  * @return : None
  */

  void run();

};
