#!/bin/bash

echo "Installing dependencies for the RealsenseRecorder"

# Save current working directory
cwd=$(pwd)

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

# Python dependencies
pip install numpy

# Add the script folders into path
echo "export PATH=$PATH:$cwd/scripts/postprocess:$cwd/scripts/evaluation/EVO" >> ~/.bashrc
echo "export PYTHONPATH=${PYTHONPATH}:$cwd/scripts/postprocess" >> ~/.bashrc

# Create the script for automatically aligning the data in a directory 
echo "python3 $cwd/scripts/postprocess/assoc_rgbdi.py \${PWD}/depth.txt \${PWD}/rgb.txt \${PWD}/acc.txt \${PWD}/gyr.txt" >> scripts/postprocess/associate.sh
sudo chmod +x scripts/postprocess/associate.sh
source ~/.bashrc
