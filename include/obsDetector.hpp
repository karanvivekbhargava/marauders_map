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
 *  @file    obsDetector.hpp
 *  @author  Karan Vivek Bhargava
 *  @copyright MIT License
 *
 *  @brief Final Project - Marauders Map - Mapping an unknown environment
 *
 *  @section DESCRIPTION
 *
 *  This program will perform laserscan for the turtlebot to check for collisions
 *
 */

#ifndef INCLUDE_OBSDETECTOR_HPP_
#define INCLUDE_OBSDETECTOR_HPP_

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"

/**
 * @brief      Class for ObsDetector.
 */
class ObsDetector {
 private:
  // Declare a varible to detect possible collisions
  bool collisionFlag_;
  // Self diagnostic
  bool diagnostic_;
  // Create a node handle
  ros::NodeHandle n_;
  // Subscribe to the laserscan topic to get obstacles
  ros::Subscriber sub_;
  // Intermediate subcriber and publisher for the minimum distance
  ros::Subscriber distanceSub_;
  ros::Publisher distancePub_;

 public:
  /**
   * @brief      Constructor for object
   */
  ObsDetector();
  /**
   * @brief      Destroys the object.
   */
  ~ObsDetector();
  /**
   * @brief      Callback function for laserscan
   *
   * @param[in]  msg   The message
   */
  void callback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * @brief      Callback function for minimum distance
   *
   * @param[in]  msg   The message
   */
  void callbackfloat(const std_msgs::Float64::ConstPtr& msg);

  /**
   * @brief      Checks for obstacles nearby
   *
   * @return     Boolean value 1 if any obstacles are nearby,
   *             0 otherwise
   */
  bool checkObstacle();

  /**
   * @brief      Gives the diagnostic
   *
   * @return     Gives self diagnostic boolean variables
   */
  bool selfDiagnosticTest();
};

#endif  // INCLUDE_OBSDETECTOR_HPP_
