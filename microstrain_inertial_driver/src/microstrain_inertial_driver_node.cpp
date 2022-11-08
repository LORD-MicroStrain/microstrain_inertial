/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Driver Definition File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
//
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include "microstrain_inertial_driver/microstrain_inertial_driver.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "microstrain_inertial_node");
  microstrain::Microstrain ustrain;
  return ustrain.run();
}
