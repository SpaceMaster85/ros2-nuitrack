README
======
A ROS2 publisher for Nuitrack pose tracking. 

This is a fork of the original package. The original package was for ROS2 Dashing and is outdated. 

The actual version has been tested on Ubuntu 20.04 with ROS2 FOXY with the Intel Realsense D435. It also forwards the Pointcloud and RGB images as rostopics


Setup
-----

This installation process is dicey.  I was able to set up the Nuitrack on an Intel NUC, twice, but not on a Lenovo laptop.  For questions, the most useful resource is probably [Nuitrack's community forums](https://community.nuitrack.com/).

1. Things you'll need:
    - Linux PC
    - Intel Realsense d435
    - A Nuitrack license (free version will work, but you still have to have them email it to you, can take 20 minutes or so), which you can get [here](https://nuitrack.com/)
2. Follow Nuitrack install instructions [here](http://download.3divi.com/Nuitrack/doc/Installation_page.html#install_ubuntu_sec), note that there are notes on their instructions below:
    - If you run into a confilct with `openni-utils` when you try to use `dpkg` to install Nuitrack, you should remove `openni-utils` with

            sudo apt-get remove openni-utils

        then install Nuitrack

            sudo dpkg -i <nuitrack package>

    - Make sure that `NUITRACK_HOME` and `LD_LIBRARY_PATH` include your a path to Nuitrack.  It tells you to put them in a script in `init.d`, this hasn't worked for me; just put them at the end of your `~/.bashrc` (or `~/.zshrc` if you use zsh).

        [Developers area - Intel RealSense SDK 2.0 - Depth and Tracking cameras](https://www.intelrealsense.com/developers/)

3. Allegedly, Nuitrack comes with drivers for the Intel Realsense, but this hasn't worked for me.  Feel free to skip this step and come back if you cannot create a camera object.  You can test this by running `nuitrack_c11_sample`, if you are unable to access the camera, follow the instructions to get the Realsense drivers.  If you do install the camera, make sure to test that you can access the camera with the Realsense Viewer (`realsense_viewer` from command line).  If you have errors, just try to enable the stereo image, anyway.

    [Developers area - Intel RealSense SDK 2.0 - Depth and Tracking cameras](https://www.intelrealsense.com/developers/)

4. Activate Nuitrack's license, from command line, from anywhere, run `nuitrack_license_tool`.
    - If the activation doesn't work, make sure that no programs accessing the camera are running.  If there are, turn them off and try activating again.
    - If you still have trouble, run the `nuitrack_sample` to get a better understanding of why the license isn't activating.  Try searching on the Nuitrack community forms for a solution.

        [Nuitrack](https://community.nuitrack.com/)

    - Afterwards, you should be able to run `nuitrack_sample` to see that things are working.
5. Install Nuitrack's SDK to a known location (you'll be referencing it in any project that builds with Nuitrack)
    - Test that you can build a project with Nuitrack.

            cd <NuitrackSDK>/Examples/nuitrack_console_sample
            mkdir build
            cd build
            cmake ..
            make
            nuitrack

6. You should see now the nuitrack demo application

If not and you get an error regarding the library path try this:

        echo 'export NUITRACK_HOME=/usr/etc/nuitrack' | sudo tee -a /etc/profile.d/nuitrack_env.sh
        echo 'export LD_LIBRARY_PATH=/usr/local/lib/nuitrack' | sudo tee -a /etc/profile.d/nuitrack_env.sh
        . /etc/profile.d/nuitrack_env.sh


7. That's it!  Now Nuitrack should work!  Now, you just need to link to the includes directory in the SDK to build a project that uses Nuitrack.

Note, that I was unable to get Nuitrack to run with ROS1, because of a Boost library conflict (probably solvable for someone more knowledgable with CMake), and I was unable to activate Nuitrack's license on a Docker container.

> Be sure to change the path to Nuitrack's SDK in `nuitrack_app/CMakeLists.txt` (see line 8).


Running it
----------

1. Make sure that you have ROS2 Foxy installed, if not, see [here](https://index.ros.org/doc/ros2/Installation/)

1. Source your ROS environment

        source /opt/ros/foxy/setup.bash

1. Make a ROS2 workspace (skip if you already have a ROS2 workspace that you'd like to use)

        mkdir -p ~/ros2_ws/src # or anywhere else you'd like
        cd ~/ros2_ws
        colcon build

1. Clone this repository into your ROS2 source directory.

        cd ~/ros2_ws/src
        git clone https://github.com/SpaceMaster85/ros2-nuitrack.git

1. Change the `nuitrack_app/CMakeLists.txt` to point to your Nuitrack SDK includes folder (see line 8)

1. Build the project

        cd ~/ros2_ws
        colcon build --symlink-install

1. Source your current workspace

        source ~/ros2_ws/install/setup.bash

To test that it works, use the following command `ros2 run nuitrack_app publisher`.  From another terminal, you can echo the topics to see that things are working okay, specifically:

    # In terminal 1
    source /opt/ros/foxy/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 run nuitrack_app publisher
    
    # In terminal 2
    ros2 topic echo /Skeletons
    
    
Warning!
Nuitrack Package installs its own (and very old) version of the libusb library! This can cause compiling issues when you compile something which needs a newer version of the libsub library. If you get compiling errors similar to "reference not found libusb_set_option" this may be the problem. I had it when I compiled rtabmap ros for ROS2.
