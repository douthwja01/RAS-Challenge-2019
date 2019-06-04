### INSTALLING LIBFREENECT2
echo "Navigating to home"
cd
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2

echo "Installing build tools"
sudo apt-get -y install build-essential cmake pkg-config

echo "Installing libusb"
sudo apt-get -y install libusb-1.0-0-dev

echo "Installing TurboJPEG"
sudo apt-get -y install libturbojpeg libjpeg-turbo8-dev

echo "Installing OpenGL"
sudo apt-get -y install libglfw3-dev

echo "Building ..."
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 -DENABLE_CXX11=ON
make
make install

echo "Setting up udev rules"
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/

###

### INSTALLING IAI_KINECT2

cd ~/catkin_ws/src/
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"
