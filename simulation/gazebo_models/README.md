# Install Gazebo
1 Make sure you have removed the Ubuntu pre-compiled binaries before installing from source:      

    sudo apt-get remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*'
    
2 Download gazebo.sh       
    change GZ_VER=7 if you want another version at gazebo.sh

3 After:

    chmod +x gazebo.sh
    ./gazebo.sh

# Warnings 
gtk-warning ** unable to locate theme engine in module_path adwaita

    sudo apt install gnome-themes-standard

# Connect to ROS
Before attempting to install the gazebo_ros_pkgs, make sure the stand-alone Gazebo works by running in terminal:

    gazebo

Test that you have the right version of Gazebo:

    which gzserver
    which gzclient
    
It should say:

    /usr/bin/gzserver
    /usr/bin/gzclient

ROS Kinetic

Kinetic is using the gazebo 7.x series, start by installing it:

    sudo apt-get install -y libgazebo7-dev

Download the source code from the gazebo_ros_pkgs github repository:

    cd ~/catkin_ws/src
    git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b kinetic-devel

Check for any missing dependencies using rosdep:

    rosdep update
    rosdep check --from-paths . --ignore-src --rosdistro kinetic

You can automatically install the missing dependencies using rosdep via debian install:

    rosdep install --from-paths . --ignore-src --rosdistro kinetic -y

Build the gazebo_ros_pkgs    
To build the Gazebo ROS integration packages, run the following commands:

    cd ~/catkin_ws/
    catkin_make

Install the protobuf library, which is used as interface to Gazebo:

    sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev gazebo7 libgazebo7-dev libxml2-utils python-rospkg python-jinja2
# Add sky

At empty.world add:

    <scene>
      <sky>
        <clouds>
            <speed>12</speed>
        </clouds>
      </sky>
    </scene>

# Links

1. http://gazebosim.org/tutorials?tut=install_from_source
3. http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install
2. https://askubuntu.com/questions/774664/gtk-warning-unable-to-locate-theme-engine-in-module-path-adwaita-error-o
3. https://github.com/ethz-asl/rotors_simulator
