/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <signal.h>

#include <ctime>
#include <string>
#include <vector>
#include <algorithm>
#include <functional>

#include <ros/callback_queue.h>
#include <tf2/LinearMath/Transform.h>

#include "microstrain_inertial_driver/microstrain_inertial_driver.h"
#include "microstrain_inertial_driver/microstrain_diagnostic_updater.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization
/////////////////////////////////////////////////////////////////////////////////////////////////////

namespace microstrain
{

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Run Function
/////////////////////////////////////////////////////////////////////////////////////////////////////
int Microstrain::run()
{
  // ROS setup
  ros::Time::init();
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // Configure the device and setup publishers/services/subscribers
  if (!NodeCommon::initialize(&node))
  {
    ROS_FATAL("Unable to initialize node base");
    return 1;
  }
  if (!NodeCommon::configure(&private_nh))
  {
    ROS_FATAL("Unable to configure node base");
    return 2;
  }
  if (!NodeCommon::activate())
  {
    ROS_FATAL("Unable to activate node base");
    return 3;
  }

  // Start the timers that will do the actual processing
  main_parsing_timer_ = create_timer<NodeCommon>(&node, timer_update_rate_hz_,
    &NodeCommon::parseAndPublishMain, this);

  // Start the aux timer if we were requested to do so
  if (config_.publish_nmea_ && config_.aux_device_ != nullptr)
  {
    ROS_INFO("Starting aux port parsing");
    aux_parsing_timer_ = create_timer<NodeCommon>(&node, 2.0,
      &NodeCommon::parseAndPublishAux, this);
  }

  // Spin until we are shutdown
  int status = 0;  // Success status. If we fail at any point this will be set to a positive number and returned
  try
  {
    ROS_INFO("Starting Data Parsing");
    ros::spin();
  }
  catch (std::exception& e)
  {
    status = 4;
    ROS_ERROR("Error: %s", e.what());
  }

  // Deactivate and shutdown
  if (!NodeCommon::deactivate())
  {
    status = 5;
    ROS_ERROR("Error while deactivating node");
  }
  if (!NodeCommon::shutdown())
  {
    status = 5;
    ROS_ERROR("Error while shutting down node");
  }

  return status;
}

}  // namespace microstrain
