README
======

A ROS2 publisher for Nuitrack pose tracking.  Currently this has been tested on Ubuntu 18.04 with ROS2 Dashing with the Intel Realsense D435. This README will become more detailed as I set things up on additional computers.

Setup
-----
Follow instructions [here](http://download.3divi.com/Nuitrack/doc/Installation_page.html#install_ubuntu_sec) to install Nuitrack.  

On 18.04, make sure that you install `libpng12-0`, and when you install install nuitrack with `dpkg` if you get a conflict with `openni-utils`, you can remove `openni-utils` with `sudo apt remove openni-utils`. 

> Don't forget to run `nuitrack_license_tool` to activate your sensor.  

You can check that Nuitrack is setup correctly by running `nuitrack_sample` and then trying to build and run the examples in the Nuitrack SDK.

> Be sure to change the path to Nuitrack's SDK in `nuitrack_app/CMakeLists.txt` (see line 8).


Running it
----------

1. Make sure that you have ROS2 Dashing installed, if not, see [here](https://index.ros.org/doc/ros2/Installation/)

1. Source your ROS environment

        source /opt/ros/dashing/setup.bash

1. Make a ROS2 workspace (skip if you already have a ROS2 workspace that you'd like to use)

        mkdir -p ~/ros2_ws/src # or anywhere else you'd like
        cd ~/ros2_ws
        colcon build

1. Clone this repository into your ROS2 source directory.

        cd ~/ros2_ws/src
        git clone https://github.com/audrow/ros2_nuitrack

1. Change the `nuitrack_app/CMakeLists.txt` to point to your Nuitrack SDK includes folder (see line 8)

1. Build the project

        cd ~/ros2_ws
        colcon build --symlink-install

1. Source your current workspace

        source ~/ros2_ws/install/setup.bash

To test that it works, use the following command `ros2 run nuitrack_app publisher`.  From another terminal, you can echo the topics to see that things are working okay; for example:

        source /opt/ros/dashing/setup.bash
        source ~/ros2_ws/install/setup.bash
        
        ros2 topic echo /Skeletons
