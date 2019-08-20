# PX4

# Install PX4

To install the development toolchain:
1. Download https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_gazebo.sh        
2. Run the script in a bash shell:

        source ubuntu_sim_ros_gazebo.sh

    You may need to acknowledge some prompts as the script progresses.

Note:

   ROS is installed with Gazebo7 by default (we have chosen to use the default rather than Gazebo8 or Gazebo9 to simplify ROS development).      
   Your catkin (ROS build system) workspace is created at ~/catkin_ws/.

# Install MAVROS

## Required dependencies

Most of the ROS dependencies are supported and installed by rosdep, including external libraries as Eigen and Boost.

GeographicLib can be installed by apt-get and it is already included on the rosdep of MAVROS package. It is also possible to compile it and install it from src but be advised to have the proper install directories the same as the ones of the apt-get install, in order to make sure that the FindGeographicLib.cmake finds the required shared libraries (libGeographic.so).

Since GeographicLib requires certain datasets (mainly the geoid dataset) so to fulfill certain calculations, these need to be installed manually by the user using geographiclib-tools, which can be installed by apt-get in Debian systems. For a quicker procedure, just run the available script in the **"mavros/scripts" folder, install_geographiclib_datasets.sh.**

Note that if you are using an older MAVROS release source install and want to update to a new one, remember to run rosdep update before running rosdep install --from-paths ${ROS_WORKSPACE} --ignore-src --rosdistro=${ROSDISTRO}, with ROS_WORKSPACE your src folder of catkin workspace. This will allow updating the rosdep list and install the required dependencies when issuing rosdep install.

**bangbang The geoid dataset is mandatory to allow the conversion between heights in order to respect ROS msg API. Not having the dataset available will shutdown the mavros_node bangbang**

exclamationRun install_geographiclib_datasets.sh to install all datasets or geographiclib-datasets-download egm96_5 (Debian 7, Ubuntu 14.04, 14.10), geographiclib-get-geoids egm96-5 (Debian 8, Fedora 22, Ubuntu 15.04 or later) to install the geoid dataset onlyexclamation

Use wstool utility for retrieving sources and catkin tool for build.

NOTE: The source installation instructions are for the ROS Kinetic release.

    sudo apt-get install python-catkin-tools python-rosinstall-generator -y

## 1. Create the workspace: unneded if you already has workspace

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin init
    wstool init src

## 2. Install MAVLink

    rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall

## 3. Install MAVROS: get source (upstream - released)
    rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall

## 4. Create workspace & deps
    wstool merge -t src /tmp/mavros.rosinstall
    wstool update -t src -j4
    rosdep install --from-paths src --ignore-src -y

## 5. Install GeographicLib datasets:
    ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

## 6. Build source
    catkin build
    source devel/setup.bash
    
# Links

1. https://github.com/mavlink/mavros/tree/master/mavros
