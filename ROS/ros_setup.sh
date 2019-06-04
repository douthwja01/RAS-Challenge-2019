### INSTALLING ROS KINETIC
echo "Setting up sources.list"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

echo "Setting up keys"
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

echo "Updating & Upgrading"
sudo apt-get update
sudo apt-get upgrade

echo "Installing ROS Kinetic"
sudo apt-get -y install ros-kinetic-desktop-full

echo "Initializing rosdep"
sudo rosdep init
rosdep update

echo "Setting up the environment"
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Instaling dependencies for building packages"
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

###

### CREATING ROS WORKSPACE
echo "Navigating to home"
cd

echo "Creating a ROS Workspace"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

echo "Sourcing catkin"
cd
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

###

### INSTALLING ROS INDUSTRIAL PACKAGE
echo "Installing Industrial Core"
sudo apt-get -y install ros-kinetic-industrial-core

echo "Installing ABB Stack"
sudo apt-get -y install ros-kinetic-abb

echo "Installing Universal Robot Stack"
sudo apt-get -y install ros-kinetic-universal-robot

echo "Installing ROS Canopen Stack"
sudo apt-get -y install ros-kinetic-ros-canopen

###

### INSTALLING ABB SPECIFIC PACKS

echo "Installing ABB Specific packs"
cd ~/catkin_ws/src
git clone -b kinetic https://github.com/ros-industrial/industrial_core.git
git clone -b kinetic-devel https://github.com/ros-industrial/abb.git
git clone -b kinetic-devel https://github.com/ros-industrial/abb_experimental.git
cd ~/catkin_ws/
rosdep install --from-paths src --ignore-src --rosdistro kinetic
catkin_make

### INSTALLING MOVEIT, DESCARTES, WEBSOCKET CLIENT

echo "Installing Moveit packages, Descartes and Websocket Client"
cd ~/catkin_ws/src
git clone -b kinetic-devel https://github.com/ros-planning/moveit.git
git clone -b kinetic-devel https://github.com/ros-industrial-consortium/descartes.git
sudo apt-get -y install python-pip
sudo apt-get -y install python3-pip
sudo pip install websocket-client
cd ~/catking_ws
rosdep install -y --from-paths . --ignore-src --rosdistro kinetic

echo "Installing ROS Bridge Components"
sudo apt-get -y install ros-kinetic-rosbridge-server

echo "Installing ROS RQT"
sudo apt-get -y install ros-kinetic-rqt
sudo apt-get -y install ros-kinetic-rqt-common-plugins
