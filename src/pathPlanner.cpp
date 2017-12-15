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
 *  @file    pathPlanner.cpp
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

#include "pathPlanner.hpp"

/**
 * @brief      Constructs the object.
 */
PathPlanner::PathPlanner() {
  ROS_INFO("Creating the walker behaviour...");
  // Set some parameters
  linSpeed_ = 0.05;
  turnSpeed_ = 0.05;
  // Publish the velocity to cmd_vel_mux/input/navi
  velocityPub_ = n_.advertise <geometry_msgs::Twist> ("/cmd_vel_mux/input/navi",
    1000);
  // Define the initial velocity message
  msg_.linear.x = 0.0;
  msg_.linear.y = 0.0;
  msg_.linear.z = 0.0;
  msg_.angular.x = 0.0;
  msg_.angular.y = 0.0;
  msg_.angular.z = 0.0;
  // Stop the turtlebot
  velocityPub_.publish(msg_);
  // Set the self diagnostic variable
  diagnostic_ = true;
}

/**
 * @brief      Destroys the object.
 */
PathPlanner::~PathPlanner() {
  // Stop the turtlebot before exiting
  msg_.linear.x = 0.0;
  msg_.linear.y = 0.0;
  msg_.linear.z = 0.0;
  msg_.angular.x = 0.0;
  msg_.angular.y = 0.0;
  msg_.angular.z = 0.0;
  // stop the turtlebot
  velocityPub_.publish(msg_);
}

/**
 * @brief      Callback for the laser scan data
 *
 * @param[in]  msg   The message
 */
void PathPlanner::plan() {
  // Set up the publisher rate to 10 Hz
  ros::Rate loop_rate(10);
  // Keep running till ros is running fine
  while (ros::ok()) {
    // Check for obstacle
    if (obsDetector_.checkObstacle()) {
      // Obstacle encountered
      ROS_INFO("Obstacle present in path. Turning...");
      // Stop the robot
      msg_.linear.x = 0.0;
      // Turn the robot
      msg_.angular.z = turnSpeed_;
    } else {
      ROS_INFO("Moving Forward...");
      // Stop turning
      msg_.angular.z = 0.0;
      // Set forward speed of the robot
      msg_.linear.x = linSpeed_;
    }

    // Publish the twist message to anyone listening
    velocityPub_.publish(msg_);

    // "Spin" a callback in case we set up any callbacks
    ros::spinOnce();

    // Sleep for the remaining time until we hit our 10 Hz rate
    loop_rate.sleep();
  }
}

bool PathPlanner::selfDiagnosticTest() {
  return diagnostic_;
}
