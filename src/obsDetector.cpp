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
  sub_ = n_.subscribe <sensor_msgs::LaserScan> ("/scan", 500,
    &ObsDetector::callback, this);
  // Publish and subscribe to the intermediate topics
  distancePub_ = n_.advertise<std_msgs::Float64>("/min_distance", 1000);
  distanceSub_ = n_.subscribe<std_msgs::Float64>("/min_distance", 1000,
    &ObsDetector::callbackfloat, this);
  // Set the value for the self diagnostic
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
  float minDist = 100;
  for (const auto& i : msg->ranges) {
    if(i < minDist) {
      minDist = i;
    }
  }
  // ROS_INFO("%.2f", minDist);
  std_msgs::Float64 msgFloat;
  msgFloat.data = minDist;
  distancePub_.publish(msgFloat);
}
/**
 * @brief      Callback for the minimum distance topic
 *
 * @param[in]  msg   The message
 */
void ObsDetector::callbackfloat(const std_msgs::Float64::ConstPtr& msg) {
  float minDist = 2;
  if ((msg->data) < minDist) {
    collisionFlag_ = true;
    return;
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

/**
* @brief      Gives the diagnostic
*
* @return     Gives self diagnostic boolean variables
*/
bool ObsDetector::selfDiagnosticTest() {
  return diagnostic_;
}
