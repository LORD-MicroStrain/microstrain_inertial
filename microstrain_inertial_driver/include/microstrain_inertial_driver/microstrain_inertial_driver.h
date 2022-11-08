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

#ifndef MICROSTRAIN_INERTIAL_DRIVER_MICROSTRAIN_INERTIAL_DRIVER_H
#define MICROSTRAIN_INERTIAL_DRIVER_MICROSTRAIN_INERTIAL_DRIVER_H

#include <cstdio>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <fstream>

#include "microstrain_inertial_driver_common/node_common.h"

namespace microstrain
{

/**
 * Implements node functionality for microstrain inertial node
 */
class Microstrain : public NodeCommon
{
public:
  /**
   * \brief Default Constructor
   */
  Microstrain() = default;

  /**
   * \brief Default Destructor
   */
  ~Microstrain() = default;

  /**
   * \brief Runs the node
   * \return 0 on success, and 1 in failure
   */
  int run();
};  // Microstrain class

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_MICROSTRAIN_INERTIAL_DRIVER_H
