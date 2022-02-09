# Install required libraries
sudo apt-get update
sudo apt-get install libusb-dev libusb-1.0-0-dev
sudo apt-get install libglfw3 libglfw3-dev
sudo apt-get install freeglut3 freeglut3-dev

# Add sources
echo 'deb https://librealsense.intel.com/Debian/apt-repo bionic main' | sudo tee
/etc/apt/sources.list.d/realsense-public.list

# The key below is incuded in the default realsense installation instructions, however some report erros with using.
# sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE

# Alexandra P. reported success with the key below
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key C8B3A55A6F3EFCDE
sudo apt-get update 
sudo apt-get install librscalibrationtool

# To test the installation
/usr/bin/Intel.Realsense.DynamicCalibrator -v
