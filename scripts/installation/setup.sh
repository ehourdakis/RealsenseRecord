
#!/bin/bash

echo "Installing the RealsenseRecorder"

# Install the glfw3 library required 
sudo apt-get install -y libeigen3-dev
sudo apt-get install -y libglfw3 libglfw3-dev 

# Needed for decompressing the camera model
sudo apt-get install -y liblz4-dev

