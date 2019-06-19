#!/bin/sh

# WARNING: This script must be ran with the 'sudo' permissions 
# to allow the installation of new packages and modification of
# system files.

# If this fails, it is likely due to network settings. for VMs, bridge adapters 
# often cause complications, consider using NAT networks.

DISTRO="kinetic"

echo "Updating apt sources..."
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
echo "Preliminary update..."
apt-get update -y
apt-get install dpkg -y
echo "Installing ROS-${DISTRO}..."
apt-get install ros-${DISTRO}-desktop-full -y

echo "Initialising rosdep..."
rosdep init
rosdep update
echo "Fixing issues associated 'sudo'"
rosdep fix-permissions

echo "Sourcing /opt/ros/.."
echo "source /opt/ros/${DISTRO}/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Installing python-install dependancies..."
apt-get install python-rosinstall -y
echo "Sourcing /opt/ros/.. directory..."
source /opt/ros/${DISTRO}/setup.bash

echo "Building catkin workspace..."
mkdir -p ~/catkin_ws/src
echo "Modifying ownerships"
chown ${SUDO_USER}:${SUDO_USER} ~/catkin_ws -R
cd ~/catkin_ws/
echo "Initial catkin build..."
catkin_make
echo "Sourcing the catkin_ws/devel/setup file."
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash

echo "Installing the ROS-${DISTRO} UR components..."
apt-get install ros-${DISTRO}-universal-robot -y

echo "The move-it commander package"
apt-get install ros-${DISTRO}-moveit -y 

echo "Final update..."
apt-get update -y 

echo "Installing the ROS-rqt tools..."
apt-get install ros-${DISTRO}-rqt -y
apt-get install ros-${DISTRO}-rqt-common-plugins -y

echo "Installing additional ROS-bridge components"
apt install python-xlib -y
apt-get install ros-${DISTRO}-rosbridge-server -y 

# Turtlebot-2 example code (connection over ROS-bridge)
apt-get install ros-${DISTRO}-turtlebot ros-${DISTRO}-turtlebot-apps ros-${DISTRO}-turtlebot-interactions ros-${DISTRO}-turtlebot-simulator ros-${DISTRO}-kobuki-ftdi ros-${DISTRO}-ar-track-alvar-msgs ros-${DISTRO}-turtlebot-gazebo -y

echo "==========================="
echo "ROS version:"
rosversion -d

echo "==> reboot when ready"

# GAZEBO FIX namespace timeout
#http://answers.gazebosim.org/question/12998/warning-waited-one-second-for-namespaces/
# Terminal 1
#roslaunch ur_gazebo ur5.launch
# Terminal 2
#roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
# Terminal 3
#roslaunch ur5_moveit_config moveit_rviz.launch config:=true
