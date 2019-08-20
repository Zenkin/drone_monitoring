#!/bin/bash

# Preventing sudo timeout https://serverfault.com/a/833888
trap "exit" INT TERM; trap "kill 0" EXIT; sudo -v || exit $?; sleep 1; while true; do sleep 60; sudo -nv; done 2>/dev/null &

sudo apt install unrar

sudo apt install ros-kinetic-joy -y

echo ""
echo "################################################################"
echo "                   Start install >>> git"
echo "################################################################"
echo ""
sleep 2
if [ -f "/usr/bin/git" ]
then
    echo " git already installed."
else
	sudo apt install git -y
fi

echo ""
echo "################################################################"
echo "                   Start install >>> htop"
echo "################################################################"
echo ""
sleep 2
if [ -f "/usr/bin/htop" ]
then
    echo " htop already installed."
else
	sudo apt install htop -y
fi


echo ""
echo "################################################################"
echo "               Start install >>> sublime text"
echo "################################################################"
echo ""
sleep 2
if [ -f "/usr/bin/subl" ]
then
    echo " subl already installed."
else
	wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
	sudo apt-get install apt-transport-https
	echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
	sudo apt-get update
	sudo apt-get install sublime-text
fi

echo ""
echo "################################################################"
echo "                    Start install >>> tmux"
echo "################################################################"
echo ""
sleep 2
if [ -f "/usr/bin/tmux" ]
then
    echo " tmux already installed."
else
	sudo apt-get install tmux
fi

# ROS Kinetic/Gazebo (ROS Kinetic includes Gazebo7 by default)
# Gazebo simulator dependencies
echo ""
echo "################################################################"
echo "                    Start install >>> ROS"
echo "################################################################"
echo ""
sleep 2 
if [ -d "/opt/ros" ]
then
    echo " ROS already installed."
else
	sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
	# ROS + Gazebo installation                                   
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
	sudo apt-get update
	sudo apt-get install ros-kinetic-desktop-full
	sudo rosdep init
	rosdep update
	echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
fi

# to GeographicLib apply certain conversions
echo ""
echo "################################################################"
echo "                  Start install >>> GeographicLib"
echo "################################################################"
echo ""
sleep 2 
sudo apt-get install libgeographic-dev
sudo apt-get install geographiclib-tools
if [[ $UID != 0 ]]; then
	echo "This script require root privileges!" 1>&2
	exit 1
fi
# Install datasets
run_get() {
	local dir="$1"
	local tool="$2"
	local model="$3"
	files=$(shopt -s nullglob dotglob; echo /usr/share/GeographicLib/$dir/$model* /usr/local/share/GeographicLib/$dir/$model*)
	if (( ${#files} )); then
		echo "GeographicLib $tool dataset $model already exists, skipping"
		return
	fi
	echo "Installing GeographicLib $tool $model"
	geographiclib-get-$tool $model >/dev/null 2>&1
}
# check which command script is available
if hash geographiclib-get-geoids; then
	run_get geoids geoids egm96-5
	run_get gravity gravity egm96
	run_get magnetic magnetic emm2015
elif hash geographiclib-datasets-download; then # only allows install the goid model dataset
	geographiclib-datasets-download egm96_5;
else
	echo "OS not supported! Check GeographicLib page for supported OS and lib versions." 1>&2
fi

echo ""
echo "################################################################"
echo "                  Start install >>> MAVROS"
echo "################################################################"
echo ""
sleep 2 

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

## Install dependencies
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools -y

## Initialise wstool
wstool init ~/catkin_ws/src

## Build MAVROS
### Get source (upstream - released)
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
### Get latest released mavlink package
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
### Setup workspace & install deps
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
if ! rosdep install --from-paths src --ignore-src --rosdistro kinetic -y; then
    # (Use echo to trim leading/trailing whitespaces from the unsupported OS name
    unsupported_os=$(echo $(rosdep db 2>&1| grep Unsupported | awk -F: '{print $2}'))
    rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --os ubuntu:xenial
fi
## Build!
catkin build
## Re-source environment to reflect new packages/build environment
catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc; 
else echo "$catkin_ws_source" >> ~/.bashrc; fi
eval $catkin_ws_source

# Upgrade cryptography
echo ""
echo "################################################################"
echo "                      Upgrade cryptography"
echo "################################################################"
echo ""
sleep 2
sudo apt install libffi-dev
sudo -H pip install --upgrade cryptography

# Ubuntu Config
echo ""
echo "################################################################"
echo "                      Remove modemmanager"
echo "################################################################"
echo ""
sleep 2
sudo apt-get remove modemmanager -y

# Common dependencies
echo ""
echo "################################################################"
echo "           Start install >>> common dependencie"
echo "################################################################"
echo ""
sleep 2
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake build-essential genromfs ninja-build exiftool vim-common -y
# Required python packages
sudo apt-get install python-argparse python-empy python-toml python-numpy python-dev python-pip -y
sudo -H pip install --upgrade pip
sudo -H pip install pandas jinja2 pyserial pyyaml
# optional python tools
sudo -H pip install pyulog

# Install FastRTPS 1.5.0 and FastCDR-1.0.7
# Real Time Publish Subscribe
fastrtps_dir=$HOME/eProsima_FastRTPS-1.5.0-Linux
echo ""
echo "################################################################"
echo "Start install >>> FastRTPS to: $fastrtps_dir"
echo "################################################################"
echo ""
sleep 2
if [ -d "$fastrtps_dir" ]
then
    echo " FastRTPS already installed."
else
    pushd .
    cd ~
    wget http://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-5-0/eprosima_fastrtps-1-5-0-linux-tar-gz -O eprosima_fastrtps-1-5-0-linux.tar.gz
    tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz eProsima_FastRTPS-1.5.0-Linux/
    tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz requiredcomponents
    tar -xzf requiredcomponents/eProsima_FastCDR-1.0.7-Linux.tar.gz
    cpucores=$(( $(lscpu | grep Core.*per.*socket | awk -F: '{print $2}') * $(lscpu | grep Socket\(s\) | awk -F: '{print $2}') ))
    cd eProsima_FastCDR-1.0.7-Linux; ./configure --libdir=/usr/lib; make -j$cpucores; sudo make install
    cd ..
    cd eProsima_FastRTPS-1.5.0-Linux; ./configure --libdir=/usr/lib; make -j$cpucores; sudo make install
    cd ..
    rm -rf requiredcomponents eprosima_fastrtps-1-5-0-linux.tar.gz
    popd
fi

# Clone PX4/Firmware
echo ""
echo "################################################################"
echo "                       Clone PX4/Firmware"
echo "################################################################"
echo ""
sleep 2
clone_dir=~/src
echo "Cloning PX4 to: $clone_dir."
if [ -d "$clone_dir" ]
then
    echo " Firmware already cloned."
else
    mkdir -p $clone_dir
    cd $clone_dir
    git clone https://github.com/PX4/Firmware.git
fi
