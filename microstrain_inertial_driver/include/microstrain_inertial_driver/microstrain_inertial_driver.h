/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Driver Definition File
// 
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2021, Parker Hannifin Corp
// 
// This code is licensed under MIT license (see LICENSE file for details)
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _MICROSTRAIN_3DM_H
#define _MICROSTRAIN_3DM_H

#include <cstdio>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <fstream>

#include "microstrain_inertial_driver_common/node_common.h"

namespace microstrain 
{

///
/// \brief Microstrain class
///
class Microstrain : public rclcpp_lifecycle::LifecycleNode, public NodeCommon
{
 public:
  Microstrain();
  ~Microstrain() = default;

  bool configure_node();
  bool activate_node();
  bool deactivate_node();
  bool shutdown_or_cleanup_node();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &prev_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &prev_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &prev_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &prev_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &prev_state);

  void parse_and_publish_main_wrapper();
  void parse_and_publish_aux_wrapper();

 private:
  void handle_exception();
}; //Microstrain class

} // namespace microstrain

#endif  // _MICROSTRAIN_3DM_GX5_45_H
