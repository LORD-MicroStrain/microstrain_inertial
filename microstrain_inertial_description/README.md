## Overview

The `microstrain_inertial_description` package contains a collection
of `.urdf.xacro` files that can be included in a user's robot
description to easily integrate MicroStrain sensors on their platform
and in the tf tree.

![All sensors in rviz](doc/all_sensors.png)

<a id="Usage"></a>

## Usage

Simply include the desired `.urdf.xacro` file in your robot's
description file and configure it's offset like so:



```
<xacro:include filename="$(find microstrain_inertial_description)/urdf/gq7.urdf.xacro" />

<xacro:gq7 parent="base_link" name="gq7_link">
  <origin xyz="0 0 0" rpy="0 0 0" />
</xacro:gq7>
```



The above example will place a GQ7 with the frame ID `gq7_link` with
it's center directly on top of `base_link`.

## Available Files

The following files can be included in other `.urdf.xacro` files to
accomodate certain sensors.

- [gq7.urdf.xacro](https://github.com/LORD-MicroStrain/microstrain_inertial/blob/ros/microstrain_inertial_description/urdf/gq7.urdf.xacro)
  can be used to represent a
  [3DM-GQ7](https://www.microstrain.com/inertial-sensors/3dm-gq7)
- [rtk.urdf.xacro](https://github.com/LORD-MicroStrain/microstrain_inertial/blob/ros/microstrain_inertial_description/urdf/gq7.urdf.xacro)
  can be used to represent a
  [3DM-RTK](https://www.microstrain.com/inertial-sensors/3dm-rtk)
- [gv7.urdf.xacro](https://github.com/LORD-MicroStrain/microstrain_inertial/blob/ros/microstrain_inertial_description/urdf/gv7.urdf.xacro)
  can be used to represent the following sensors:
- [3DM-GV7-AR](https://www.microstrain.com/inertial-sensors/3DM-GV7-AR)
- [3DM-GV7-AHRS](https://www.microstrain.com/inertial-sensors/3DM-GV7-AHRS)
- [3DM-GV7-INS](https://www.microstrain.com/inertial-sensors/3DM-GV7-INS)
- [cv7.urdf.xacro](https://github.com/LORD-MicroStrain/microstrain_inertial/blob/ros/microstrain_inertial_description/urdf/cv7.urdf.xacro)
  can be used to represent the following sensors:
- [3DM-CV7-AR](https://www.microstrain.com/inertial-sensors/3dmcv7-ar)
- [3DM-CV7-AHRS](https://www.microstrain.com/inertial-sensors/3dm-cv7-ahrs)
- [3DM-CV7-INS](https://www.microstrain.com/inertial-sensors/3DM-CV7-INS)
- [cv5.urdf.xacro](https://github.com/LORD-MicroStrain/microstrain_inertial/blob/ros/microstrain_inertial_description/urdf/cv5.urdf.xacro)
  can be used to represent the following sensors:
- [3DM-CV5-10](https://www.microstrain.com/inertial-sensors/3dm-cv5-10)
- [3DM-CV5-15](https://www.microstrain.com/inertial-sensors/3dm-cv5-15)
- [3DM-CV5-25](https://www.microstrain.com/inertial-sensors/3dm-cv5-25)
- [3DM-CX5-10](https://www.microstrain.com/inertial-sensors/3dm-cx5-10)
- [3DM-CX5-15](https://www.microstrain.com/inertial-sensors/3dm-cx5-15)
- [3DM-CX5-25](https://www.microstrain.com/inertial-sensors/3dm-cx5-25)
- [3DM-CX5-45](https://www.microstrain.com/inertial-sensors/3dm-cx5-45)
- [gx5_15.urdf.xacro](https://github.com/LORD-MicroStrain/microstrain_inertial/blob/ros/microstrain_inertial_description/urdf/gx5_15.urdf.xacro)
  can be used to represent a
  [3DM-GX5-15](https://www.microstrain.com/inertial-sensors/3dm-gx5-15)
- [gx5_25.urdf.xacro](https://github.com/LORD-MicroStrain/microstrain_inertial/blob/ros/microstrain_inertial_description/urdf/gx5_25.urdf.xacro)
  can be used to represent a
  [3DM-GX5-25](https://www.microstrain.com/inertial-sensors/3dm-gx5-25)
- [gx5_25.urdf.xacro](https://github.com/LORD-MicroStrain/microstrain_inertial/blob/ros/microstrain_inertial_description/urdf/gx5_45.urdf.xacro)
  can be used to represent a
  [3DM-GX5-25](https://www.microstrain.com/inertial-sensors/3dm-gx5-25)

## Sensor Origin

With the exception of the 3DM-RTK, all of the above mentioned sensors
have a sensor origin which will be where most measurements are sampled.
When a user places the sensor on their robot, the `origin` is the
location of the center of the sensor. The above urdf files will place a
link there called `\${name}_center`, and then place another link
offset from the center of the sensor to the sensor origin called
`\${name}`. For most sensors, this offset is barely noticable, but it
can be important if precision of measurements is important.

For example, if one were to place a GQ7 on a robot like the example in
[Usage](#Usage), they would
see the following frames available in rviz.

![GQ7 with sensor origin](doc/gq7_example.png)

The purpose of this is to make it easy to place the sensor by allowing
users to place the center of the sensor wherever they want without
having to think about the sensor origin, but then also making the
requested frame so that data reported from the sensor will be as
accurate as possible
