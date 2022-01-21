## Description

Interface (driver) software, including ROS node, for inertial sensors compatible with the [Microstrain Communication Library (MSCL)](https://github.com/LORD-MicroStrain/MSCL).

MSCL is developed by [LORD Sensing - Microstrain](http://microstrain.com) in Williston, VT. 

## Different Codebases

This repo is now structured differently as of `2.0.0`.

#### Important Branches
There are three important branches that you may want to checkout:

* [ros](https://github.com/LORD-MicroStrain/ROS-MSCL/tree/ros) -- Contains ROS1 implementation for this node as of `2.0.0`. This version is being actively updated and supported
* [ros2](https://github.com/LORD-MicroStrain/ROS-MSCL/tree/ros2) -- Contains ROS2 implementation for this node as of `2.0.0`. This version is being actively updated and supported
* [master](https://github.com/LORD-MicroStrain/ROS-MSCL/tree/master) -- Contains the most recent ROS1 changes before the transition to `2.0.0`. Kept for backwards compatibility, but no longer updated or supported

Both the `ros` and `ros2` branches share most of their code by using git submodules. The following submodules contain most of the actual implementations:

* [microstrain_inertial_driver_common](https://github.com/LORD-MicroStrain/microstrain_inertial_driver_common/tree/main) submoduled in this repo at `microstrain_inertial_driver/microstrain_inertial_driver_common`
* [microstrain_inertial_msgs_common](https://github.com/LORD-MicroStrain/microstrain_inertial_msgs_common/tree/main) submoduled in this repo at `microstrain_inertial_msgs/microstrain_inertial_msgs_common`
* [microstrain_inertial_rqt_common](https://github.com/LORD-MicroStrain/microstrain_inertial_rqt_common/tree/main) submoduled in this repo at `microstrain_inertial_rqt/microstrain_inertial_rqt_common`

#### Different Package Names

Prior to version `2.0.0`, this repo contained the following ROS packages:
* `ros_mscl` -- ROS node that will communicate with the devices
* `mscl_msgs` -- Collection of messages produced by the `ros_mscl` node
* `ros_mscl_cpp_example` -- Simple subscriber written in C++ that will consume a message produced by `ros_mscl`
* `ros_mscl_py_example` -- Simple subscriber written in Python that will consume a message produced by `ros_mscl`

Due to requirements laid out by the ROS maintainers [here](https://www.ros.org/reps/rep-0144.html), as of version `2.0.0`, this repo contains the following ROS packages:
* `microstrain_inertial_driver` -- ROS node that will communicate with the devices
* `microstrain_inertial_msgs` -- Collection of messages produces by the `microstrain_inertial_driver` node
* `microstrain_inretial_examples` -- Collection of examples that show how to interact with the `microstrain_inertial_driver` node. Currently contains one simple C++ and python subscriber node
* `microstrain_inertial_rqt` -- Collection of RQT plugins to view the status of inertial devices when running the `microstrain_inertial_driver`

## Install Instructions

### Docker

As of `v2.2.2` the `microstrain_inertial_driver` is distributed as a docker image. More information on how to use the image can be found on [DockerHub](https://hub.docker.com/r/microstrain/ros-microstrain_inertial_driver)


### Buildfarm

As of `v2.0.5` this package is being built and distributed by the ROS build farm. If you do not need to modify the source, it is recommended to install directly from the buildfarm by running the following commands where `ROS_DISTRO` is the version of ROS you are using such as `melodic` or `noetic`:

Driver:
```bash
sudo apt-get update && sudo apt-get install ros-ROS_DISTRO-microstrain-inertial-driver
```

RQT:
```bash
sudo apt-get update && sudo apt-get install ros-ROS_DISTRO-microstrain-inertial-rqt
```

For more information on the ROS distros and platforms we support, please see [index.ros.org](https://index.ros.org/r/microstrain_inertial/github-LORD-MicroStrain-microstrain_inertial/#noetic)


### Source

If you need to modify the source of this repository, or are running on a platform that we do not support, you can build from source by following these instructions

#### Submodules
This repo now takes advantage of git submodules in order to share code between ROS versions. When cloning the repo, you should clone with the `--recursive` flag to get all of the submodules.

If you have already cloned the repo, you can checkout the submodules by running `git submodule init && git submodule update --recursive` from the project directory

The [CMakeLists.txt](./microstrain_inertial_msgs/CMakeLists.txt) will automatically checkout the submodule if it does not exist, but it will not keep it up to date. In order to keep up to date, every
time you pull changes you should pull with the `--recurse-submodules` flag, or alternatively run `git submodule update --recursive` after you have pulled changes

#### MSCL
MSCL is now installed in the [CMakeLists.txt](./microstrain_inertial_driver/CMakeLists.txt). The version installed can be changed by passing the flag `-DMSCL_VERSION="62.0.0"`

If you already have MSCL installed and want to use your installed version instead of the one automatically downloaded, you can specify the location by passing the flag `-DMSCL_DIR=/usr/share/c++-mscl`

We do our best to keep ROS-MSCL up-to-date with the latest MSCL changes, but sometimes there is a delay. The currently supported version of MSCL is [v62.1.2](https://github.com/LORD-MicroStrain/MSCL/releases/tag/v62.1.2)

#### Building from source
1. Install ROS and create a workspace: [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

2. Move the entire microstrain_inertial folder (microstrain_inertial_driver, microstrain_inertial_msgs , and microstrain_common for just source) to the your_workspace/src directory.

3. Locate and register the ros_mscl package: `rospack find microstrain_inertial_driver`

4. Install rosdeps for this package: `rosdep install --from-paths ~/your_workspace/src --ignore-src -r -y`

5. Build your workspace:
        
        cd ~/your_workspace
        catkin_make
        source ~/your_workspace/devel/setup.bash
   The source command may need to be run in each terminal prior to launching a ROS node.
#### Launch the node and publish data
The following command will launch the driver. Keep in mind each instance needs to be run in a separate terminal.
            
        roslaunch microstrain_inertial_driver microstrain.launch
Optional launch parameters:
- name: namespace the node will publish messages to, default: gx5
- port: serial port name to connect to the device over, default: /dev/ttyACM0
- baudrate: baud rate to open the connection with, default: 115200
- imu_rate: sample rate for IMU data (hz), default: 100
- debug: output debug info? default: false
- diagnostics: output diagnostic info? default: true
    
To check published topics:
        
    rostopic list

**Example**: Connect to and publish data from two devices simultaneously  
In two different terminals:
    
    roslaunch microstrain_inertial_driver microstrain.launch name:=sensor1234

    roslaunch microstrain_inertial_driver microstrain.launch name:=bestSensor port:=/dev/ttyACM1
This will launch two nodes that publish data to different namespaces:
- sensor1234, connected over port: /dev/ttyACM0
- bestSensor, connected over port: /dev/ttyACM1

An example subscriber node can be found here: [Microstrain Examples](./microstrain_inertial_examples)  


## Docker Development

### VSCode

The easiest way to develop in docker while still using an IDE is to use VSCode as an IDE. Follow the steps below to develop on this repo in a docker container

1. Install the following dependencies:
    1. [VSCode](https://code.visualstudio.com/)
    1. [Docker](https://docs.docker.com/get-docker/)
1. Open VSCode and install the following [plugins](https://code.visualstudio.com/docs/editor/extension-marketplace):
    1. [VSCode Docker plugin](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)
    1. [VSCode Remote Containers plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
1. Open this directory in a container by following [this guide](https://code.visualstudio.com/docs/remote/containers#_quick-start-open-an-existing-folder-in-a-container)
    1. Due to a bug in the remote container plugin, you will need to refresh the window once it comes up. To do this, type `Ctrl+Shift+p` and type `Reload Window` and hit enter. Note that this will have to be repeated every time the container is rebuilt
1. Once the folder is open in VSCode, you can build the project by running `Ctrl+Shift+B` to trigger a build, or `Ctrl+p` to open quick open, then type `task build` and hit enter
1. You can run the project by following [this guide](https://code.visualstudio.com/docs/editor/debugging)

### Make

If you are comfortable working from the command line, or want to produce your own runtime images, the [Makefile](./devcontainer/Makefile) in the [.devcontainer](./devcontainer) 
directory can be used to build docker images, run a shell inside the docker images and produce a runtime image. Follow the steps below to setup your environment to use the `Makefile`

1. Install the following dependencies:
    1. [Make](https://www.gnu.org/software/make/)
    1. [Docker](https://docs.docker.com/get-docker/)
    1. [qemu-user-static](https://packages.ubuntu.com/bionic/qemu-user-static) (for multiarch builds)
        1. Run the following command to register the qemu binaries with docker: `docker run --rm --privileged multiarch/qemu-user-static:register`

The `Makefile` exposes the following tasks. They can all be run from the `.devcontainer` directory:
* `make build-shell` - Builds the development docker image and starts a shell session in the image allowing the user to develop and build the ROS project using common commands such as `catkin_make`
* `make image` - Builds the runtime image that contains only the required dependencies and the ROS node.
* `make clean` - Cleans up after the above two tasks

## License

Different packages in this repo are releasd under different licenses. For more information, see the LICENSE files in each of the package directories.

Here is a quick overview of the licenses used in each package:

| Package                                                                  | License |
| ------------------------------------------------------------------------ | ------- |
| [microstrain_inertial_driver](./microstrain_inertial_driver/LICENSE)     | MIT     |
| [microstrain_inertial_msgs](./microstrain_inertial_msgs/LICENSE)         | MIT     |
| [microstrain_inertial_rqt](./microstrain_inertial_rqt/LICENSE)           | BSD     |
| [microstrain_inertial_examples](./microstrain_inertial_examples/LICENSE) | MIT     |
