
#!/bin/bash

echo "Installing dependencies for the RealsenseRecorder"

# Install Boost
sudo apt-get install -y libboost-all-dev

# Install openGL
sudo apt-get install -y libglu1-mesa-dev freeglut3-dev mesa-common-dev

# Install the glfw3 library required 
sudo apt-get install -y libeigen3-dev
sudo apt-get install -y libglfw3 libglfw3-dev 

# Needed for decompressing the camera model
sudo apt-get install -y liblz4-dev

# Install Eigen
sudo apt-get install -y libeigen3-dev