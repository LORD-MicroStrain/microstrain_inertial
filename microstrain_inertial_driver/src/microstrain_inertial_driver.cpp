/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord ROS2 Inertial Driver Implementation File
// 
// Copyright (c) 2017, Brian Bingham
// Copyright (c)  2021, Parker Hannifin Corp
// 
// This code is licensed under MIT license (see LICENSE file for details)
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <algorithm>
#include <time.h>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <ctime>

#include "microstrain_inertial_driver/microstrain_inertial_driver.h"
#include <tf2/LinearMath/Transform.h>
#include "lifecycle_msgs/msg/transition.hpp"

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization
/////////////////////////////////////////////////////////////////////////////////////////////////////

namespace microstrain
{

Microstrain::Microstrain() : rclcpp_lifecycle::LifecycleNode("ros2_mscl_node")
{
  //Initialize the helper classes
  if (!MicrostrainNodeBase::initialize(this))
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize base node");
}



/////////////////////////////////////////////////////////////////////////////////////////////////////
// Configure State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Microstrain::on_configure(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
 
  if(configure_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Activate State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Microstrain::on_activate(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

  if(activate_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Deactivate State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Microstrain::on_deactivate(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

  if(deactivate_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Cleanup State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Microstrain::on_cleanup(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");

  if(shutdown_or_cleanup_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
 // Shutdown State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Microstrain::on_shutdown(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called.");

  if(shutdown_or_cleanup_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
// Configure Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::configure_node()
{
  ///////////////////////////////////////////////////////////////////////////
  //
  //Main loop setup
  ///
  ///////////////////////////////////////////////////////////////////////////
  try {
    RCLCPP_DEBUG(this->get_logger(), "Initializing base node");
    if (!MicrostrainNodeBase::configure(this))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure node base");
      return false;
    }
  }
  catch(const mscl::Error_Connection&)
  {
    RCLCPP_ERROR(this->get_logger(), "Error: Device Disconnected");
    return false;
  }

  catch(const mscl::Error &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    return false;
  }


  return true;
} 


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Activate Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::activate_node()
{
  // Activate the base node to start the background tasks
  if (!MicrostrainNodeBase::activate())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to activate base node");
    return false;
  }

  // Start a timer around a wrapper function to catch errors
  main_parsing_timer_ = create_timer<Microstrain>(node_, timer_update_rate_hz_,
      &Microstrain::parse_and_publish_main_wrapper, this);
  device_status_timer_ = create_timer<Microstrain>(node_, 1.0,
      &Microstrain::device_status_wrapper, this);

  // Start the aux timer if we were requested to do so
  if (config_.supports_rtk_ && config_.publish_nmea_)
  {
    RCLCPP_INFO(this->get_logger(), "Starting aux port parsing");
    aux_parsing_timer_ = create_timer<Microstrain>(node_, 2.0,
        &Microstrain::parse_and_publish_aux_wrapper, this);
  }

  //Activate publishers
  if (publishers_.device_status_pub_)
    publishers_.device_status_pub_->on_activate();

  //IMU Publishers
  if(publishers_.imu_pub_)
    publishers_.imu_pub_->on_activate();
   
  if(publishers_.mag_pub_)
    publishers_.mag_pub_->on_activate();

  if(publishers_.gps_corr_pub_)
    publishers_.gps_corr_pub_->on_activate();
 
 
  //GNSS Publishers
  for(int i=0; i< NUM_GNSS; i++)
  {
    if(publishers_.gnss_pub_[i])
      publishers_.gnss_pub_[i]->on_activate();
  
    if(publishers_.gnss_odom_pub_[i])
      publishers_.gnss_odom_pub_[i]->on_activate();

    if(publishers_.gnss_time_pub_[i])
      publishers_.gnss_time_pub_[i]->on_activate();

    if(publishers_.gnss_aiding_status_pub_[i])
      publishers_.gnss_aiding_status_pub_[i]->on_activate();

    if(publishers_.gnss_fix_info_pub_[i])
      publishers_.gnss_fix_info_pub_[i]->on_activate();
  }

  //RTK Data publishers
  if(publishers_.rtk_pub_)
    publishers_.rtk_pub_->on_activate();

  if(publishers_.rtk_pub_v1_)
    publishers_.rtk_pub_v1_->on_activate();
  
  //NMEA publisher
  if (publishers_.nmea_sentence_pub_)
    publishers_.nmea_sentence_pub_->on_activate();

  //Filter Publishers
  if(publishers_.filter_status_pub_)
    publishers_.filter_status_pub_->on_activate();

  if(publishers_.filter_heading_pub_)
    publishers_.filter_heading_pub_->on_activate();

  if(publishers_.filter_heading_state_pub_)
    publishers_.filter_heading_state_pub_->on_activate();

  if(publishers_.filter_pub_)
    publishers_.filter_pub_->on_activate();

  if(publishers_.filtered_imu_pub_)
    publishers_.filtered_imu_pub_->on_activate();

  if(publishers_.filter_relative_pos_pub_)
    publishers_.filter_relative_pos_pub_->on_activate();
  
  if(publishers_.filter_aiding_measurement_summary_pub_)
    publishers_.filter_aiding_measurement_summary_pub_->on_activate();

  if(publishers_.gnss_dual_antenna_status_pub_)
    publishers_.gnss_dual_antenna_status_pub_->on_activate();
  
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Deactivate Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::deactivate_node()
{
  //Deactivate the base node
  if (!MicrostrainNodeBase::deactivate())
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to deactivate node base");
  }

  //Deactivate publishers
  if (publishers_.device_status_pub_)
    publishers_.device_status_pub_->on_deactivate();
 
  //IMU Publishers
  if(publishers_.imu_pub_)
    publishers_.imu_pub_->on_deactivate();
   
  if(publishers_.mag_pub_)
    publishers_.mag_pub_->on_deactivate();

  if(publishers_.gps_corr_pub_)
    publishers_.gps_corr_pub_->on_deactivate();
 
 
  //GNSS Publishers
  for(int i=0; i< NUM_GNSS; i++)
  {
    if(publishers_.gnss_pub_[i])
      publishers_.gnss_pub_[i]->on_deactivate();
  
    if(publishers_.gnss_odom_pub_[i])
      publishers_.gnss_odom_pub_[i]->on_deactivate();

    if(publishers_.gnss_time_pub_[i])
      publishers_.gnss_time_pub_[i]->on_deactivate();

    if(publishers_.gnss_aiding_status_pub_[i])
      publishers_.gnss_aiding_status_pub_[i]->on_deactivate();

    if(publishers_.gnss_fix_info_pub_[i])
      publishers_.gnss_fix_info_pub_[i]->on_deactivate();
  }

  //RTK Data publishers
  if(publishers_.rtk_pub_)
    publishers_.rtk_pub_->on_deactivate();

  if(publishers_.rtk_pub_v1_)
    publishers_.rtk_pub_v1_->on_deactivate();
  
  //Filter Publishers
  if(publishers_.filter_status_pub_)
    publishers_.filter_status_pub_->on_deactivate();

  if(publishers_.filter_heading_pub_)
    publishers_.filter_heading_pub_->on_deactivate();

  if(publishers_.filter_heading_state_pub_)
    publishers_.filter_heading_state_pub_->on_deactivate();

  if(publishers_.filter_pub_)
    publishers_.filter_pub_->on_deactivate();

  if(publishers_.filtered_imu_pub_)
    publishers_.filtered_imu_pub_->on_deactivate();

  if(publishers_.filter_relative_pos_pub_)
    publishers_.filter_relative_pos_pub_->on_deactivate();

  if(publishers_.filter_aiding_measurement_summary_pub_)
    publishers_.filter_aiding_measurement_summary_pub_->on_deactivate();

  if(publishers_.gnss_dual_antenna_status_pub_)
    publishers_.gnss_dual_antenna_status_pub_->on_deactivate();

  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Shutdown/Cleanup Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::shutdown_or_cleanup_node()
{
  //Shutdown the base node
  if(!MicrostrainNodeBase::shutdown())
  {
    // Even though this is an error, don't return a failure on shutdown as it is not a fatal error
    RCLCPP_ERROR(this->get_logger(), "Failed to shutdown base node");
  }

  //Release publishers
  if (publishers_.device_status_pub_)
    publishers_.device_status_pub_.reset();

  //IMU Publishers
  if(publishers_.imu_pub_)
    publishers_.imu_pub_.reset();
   
  if(publishers_.mag_pub_)
    publishers_.mag_pub_.reset();

  if(publishers_.gps_corr_pub_)
    publishers_.gps_corr_pub_.reset();
 
 
  //GNSS Publishers
  for(int i=0; i< NUM_GNSS; i++)
  {
    if(publishers_.gnss_pub_[i])
      publishers_.gnss_pub_[i].reset();
  
    if(publishers_.gnss_odom_pub_[i])
      publishers_.gnss_odom_pub_[i].reset();

    if(publishers_.gnss_time_pub_[i])
      publishers_.gnss_time_pub_[i].reset();

    if(publishers_.gnss_aiding_status_pub_[i])
      publishers_.gnss_aiding_status_pub_[i].reset();

    if(publishers_.gnss_fix_info_pub_[i])
      publishers_.gnss_fix_info_pub_[i].reset();
  }

  //RTK Data publishers
  if(publishers_.rtk_pub_)
    publishers_.rtk_pub_.reset();

  if(publishers_.rtk_pub_v1_)
    publishers_.rtk_pub_v1_.reset();
  
  //Filter Publishers
  if(publishers_.filter_status_pub_)
    publishers_.filter_status_pub_.reset();

  if(publishers_.filter_heading_pub_)
    publishers_.filter_heading_pub_.reset();

  if(publishers_.filter_heading_state_pub_)
    publishers_.filter_heading_state_pub_.reset();

  if(publishers_.filter_pub_)
    publishers_.filter_pub_.reset();

  if(publishers_.filtered_imu_pub_)
    publishers_.filtered_imu_pub_.reset();

  if(publishers_.filter_relative_pos_pub_)
    publishers_.filter_relative_pos_pub_.reset();

  if(publishers_.filter_aiding_measurement_summary_pub_)
    publishers_.filter_aiding_measurement_summary_pub_.reset();

  if(publishers_.gnss_dual_antenna_status_pub_)
    publishers_.gnss_dual_antenna_status_pub_.reset();

  return true;
}

void Microstrain::parse_and_publish_main_wrapper()
{
  // call the parsing function in a try catch block so we can transition the state instead of crashing when an error happens
  try
  {
    parseAndPublishMain();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error during main processing: %s", e.what());
    handle_exception();
  }
}

void Microstrain::parse_and_publish_aux_wrapper()
{
  // call the parsing function in a try catch block so we can transition the state instead of crashing when an error happens
  try
  {
    parseAndPublishAux();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error during aux processing: %s", e.what());
    handle_exception();
  }
}

void Microstrain::device_status_wrapper()
{
  // call the device status function in a try catch block so we can transition the state instead of crashing when an error happens
  try
  {
    publishers_.publishDeviceStatus();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error during processing: %s", e.what());
    handle_exception();
  }
}

void Microstrain::handle_exception()
{
  // Manuallly transition to deactivate state so that the node can be cleanly restarted
  RCLCPP_INFO(this->get_logger(), "Transitioning to deactivate state");
  const auto& inactive_state = LifecycleNode::deactivate();
  if (inactive_state.label() == "inactive")
  {
    RCLCPP_WARN(this->get_logger(), "Successfully transitioned to inactive, cleaning up node to fresh state");
    const auto& cleanup_state = LifecycleNode::cleanup();
    if (cleanup_state.label() == "unconfigured")
    {
      RCLCPP_WARN(this->get_logger(), "Node has been successfully cleaned up from error. transition to configure state to reconfigure");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Transition to cleanup resulted in transition to %s instead of inactive", cleanup_state.label().c_str());
      RCLCPP_ERROR(this->get_logger(), "Unable to recover, so transitioning to shutdown. This node is no longer usable");
      LifecycleNode::shutdown();
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Transition to deactivate resulted in transition to %s instead of inactive", inactive_state.label().c_str());
    RCLCPP_ERROR(this->get_logger(), "Unable to recover, so transitioning to shutdown. This node is no longer usable");
    LifecycleNode::shutdown();
  }
}

} // namespace Microstrain
