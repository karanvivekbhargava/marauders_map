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
 *  @file    talkerTest.cpp
 *  @author  Karan Vivek Bhargava
 *  @copyright MIT License
 *
 *  @brief Assignment to implement walker behavior for turtlebot
 *
 *  @section DESCRIPTION
 *
 *  This program will test the walker node for the turtlebot walker
 *
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "pathPlanner.hpp"
#include "obsDetector.hpp"

class TestClass{
 public:
  void dummyCallBack(const std_msgs::Float64::ConstPtr& msg) {}
};

/**
 * @brief      Tests whether collision flag is set to false
 *
 * @param[in]  TESTSuite          gtest framework
 * @param[in]  sanityCheck        Name of the test
 */
TEST(TESTSuite, PathPlannerSelfDiagnosticTest) {
  PathPlanner pathPlanner;
  EXPECT_EQ(pathPlanner.selfDiagnosticTest() , true);
}

TEST(TESTSuite, IntializationErrorTest) {
  ros::NodeHandle n_;
  EXPECT_NO_FATAL_FAILURE(PathPlanner pathPlanner);
}

TEST(TESTSuite, PathPlannerPublisherTest) {
  ros::NodeHandle n_;
  TestClass t;
  ros::Subscriber sub = n_.subscribe("/min_distance", 1, &TestClass::dummyCallBack, &t);
  ros::WallDuration(1).sleep();
  EXPECT_EQ(sub.getNumPublishers(), 1);
}

TEST(TESTSuite, PathPlannerSubscriberTest) {
  ros::NodeHandle n_;
  ros::Publisher pub = n_.advertise<std_msgs::Float64>("/min_distance", 0);
  ros::WallDuration(1).sleep();
  EXPECT_EQ(pub.getNumSubscribers(), 1);
}
