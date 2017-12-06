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
 *  @file    obsDetector.cpp
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

#include "obsDetector.hpp"

/**
 * @brief      Constructs the object.
 */
ObsDetector::ObsDetector() {
  ROS_INFO("Creating the obstacle detection behaviour...");
  // initialise the collision flag to be false
  collisionFlag_ = false;
  // Subcribe to the /scan topic and use the laserCallback method
  sub = n.subscribe <sensor_msgs::LaserScan> ("/scan", 500,
    &ObsDetector::callback, this);

  diagnostic_ = true;
}

/**
 * @brief      Destroys the object.
 */
ObsDetector::~ObsDetector() {
}

/**
 * @brief      Callback for the laser scan data
 *
 * @param[in]  msg   The message
 */
void ObsDetector::callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  for (int i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] < 1.2) {
      collisionFlag_ = true;
      return;
    }
  }
  collisionFlag_ = false;
}

/**
 * @brief      Returns the collision flag
 *
 * @return     boolean value for the collision flag
 */
bool ObsDetector::checkObstacle() {
  return collisionFlag_;
}

bool ObsDetector::selfDiagnosticTest() {
  return diagnostic_;
}
