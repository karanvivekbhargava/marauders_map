/**
 *  MIT License
 *
 *  Copyright (c) 2017 Karan Vivek Bhargava
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
 *  DEALINGS IN THE SOFTWARE.
 *
 *  @file    pathPlanner.hpp
 *  @author  Karan Vivek Bhargava
 *  @copyright MIT License
 *
 *  @brief Final Project - Marauders Map - Mapping an unknown environment
 *
 *  @section DESCRIPTION
 *
 *  This program will perform path planning operations for the turtlebot to check for collisions
 *
 */

#ifndef INCLUDE_PATHPLANNER_HPP_
#define INCLUDE_PATHPLANNER_HPP_

#include "geometry_msgs/Twist.h"
#include "obsDetector.hpp"

/**
 * @brief      Class for PathPlanner.
 */
class PathPlanner {
 private:
  // Create ObsDetector
  ObsDetector obsDetector_;
  // Self diagnostic
  bool diagnostic_;
  // Declare a variable for the velocities
  geometry_msgs::Twist msg_;
  // Create a node handle
  ros::NodeHandle n_;
  // Publish the "velocity" topic to the turtlebot
  ros::Publisher velocityPub_;
  // Define the linear and turn speeds
  float linSpeed_;
  float turnSpeed_;

 public:
  /**
   * @brief      Constructor for object
   */
  PathPlanner();
  /**
   * @brief      Destroys the object.
   */
  ~PathPlanner();
  /**
   * @brief      Plans the next action
   */
  void plan();
  /**
   * @brief      Gives the diagnostic
   *
   * @return     Gives self diagnostic boolean variables
   */
  bool selfDiagnosticTest();
};

#endif  // INCLUDE_PATHPLANNER_HPP_
