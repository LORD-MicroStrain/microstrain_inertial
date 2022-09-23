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

#include "mscl/mscl.h"
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
  if (!MicrostrainNodeBase::initialize(&node))
    ROS_FATAL("Unable to initialize node base");
  if (!MicrostrainNodeBase::configure(&private_nh))
    ROS_FATAL("Unable to configure node base");
  if (!MicrostrainNodeBase::activate())
    ROS_FATAL("Unable to activate node base");

  // Start the timers that will do the actual processing
  main_parsing_timer_ = create_timer<MicrostrainNodeBase>(&node, timer_update_rate_hz_,
    &MicrostrainNodeBase::parseAndPublishMain, this);
  device_status_timer_ = create_timer<MicrostrainPublishers>(&node, 1.0,
    &MicrostrainPublishers::publishDeviceStatus, &publishers_);

  // Start a timer to log debug information if debug is enabled
  if (config_.debug_)
    log_debug_timer_ = create_timer<MicrostrainNodeBase>(&node, 1.0,
      &MicrostrainNodeBase::logDeviceDebugInfo, this);

  // Start the aux timer if we were requested to do so
  if (config_.supports_rtk_ && config_.publish_nmea_)
  {
    ROS_INFO("Starting aux port parsing");
    aux_parsing_timer_ = create_timer<MicrostrainNodeBase>(&node, 2.0,
      &MicrostrainNodeBase::parseAndPublishAux, this);
  }

  // Spin until we are shutdown
  int status = 0;  // Success status. If we fail at any point this will be set to 1 and returned
  try
  {
    ROS_INFO("Starting Data Parsing");
    ros::spin();
  }
  catch (mscl::Error_Connection)
  {
    status = 1;
    ROS_ERROR("Device Disconnected");
  }
  catch (mscl::Error& e)
  {
    status = 1;
    ROS_ERROR("Error: %s", e.what());
  }

  // Deactivate and shutdown
  if (!MicrostrainNodeBase::deactivate())
  {
    status = 1;
    ROS_ERROR("Error while deactivating node");
  }
  if (!MicrostrainNodeBase::shutdown())
  {
    status = 1;
    ROS_ERROR("Error while shutting down node");
  }

  return status;
}

}  // namespace microstrain
