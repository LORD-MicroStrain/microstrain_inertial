#!/bin/bash

set -e

# Source the ROS setup bash
source /opt/ros/${DOCKER_ROS_VERSION}/setup.bash

# Run roslaunch with whatever params were passed
exec roslaunch microstrain_inertial_driver microstrain.launch $@