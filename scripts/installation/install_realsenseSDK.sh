#!/bin/bash
# Installs the realsense SDK from source

# Create the dirs
mkdir realsense && cd realsense && mkdir install

# Save current working directory
cwd=$(pwd)

# Install the essentials
sudo apt-get install -y libusb-1.0-0-dev libssl-dev
sudo apt-get install -y x11-apps libx11-dev
sudo apt-get install -y xorg-dev libglu1-mesa-dev

# Checkout and build the latest realsense SDK
REALSENSE_HEAD=v2.50.0
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout $REALSENSE_HEAD

mkdir build && cd build

cmake -DCMAKE_INSTALL_PREFIX:PATH="/files/Libraries/realsense/install" ..

make
make install

# Save the install library path
echo "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/files/Libraries/realsense/install/lib/" >> ~/.bashrc
echo "export PATH=$PATH:/files/Libraries/realsense/install/bin" >> ~/.bashrc
source ~/.bashrc

# Setup the udev rules for the device permissions
cd "$cwd"/librealsense
source scripts/setup_udev_rules.sh

