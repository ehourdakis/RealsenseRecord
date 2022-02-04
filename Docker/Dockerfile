# Docker file for interfacing with realsense cameras

FROM ubuntu:18.04 

ARG build_type

# Switch to user root
USER root
ENV DEBIAN_FRONTEND=noninteractive

# First update
RUN apt-get -y update
RUN apt-get -y upgrade

RUN apt-get install -y gnupg  lsb-core 
RUN apt-get install -y software-properties-common

# Add realsense repo
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# Install realsense + drivers
RUN apt-get install -y librealsense2-dkms
RUN apt-get install -y librealsense2-utils
RUN apt-get install -y librealsense2-dev
RUN apt-get install -y librealsense2-dbg

# Install the nvidia drivers (so that OGL apps can run consistently)
RUN if [ "$build_type" = "GUI" ] ; then \
      add-apt-repository ppa:graphics-drivers/ppa; \
      apt update && apt install -y nvidia-driver-470; \
    fi

# Install the dependencies of the RealsenseRecorder
# Install Boost
RUN apt-get install -y libboost-all-dev

# Install openGL
RUN apt-get install -y libglu1-mesa-dev freeglut3-dev mesa-common-dev

# Install the glfw3 library required 
RUN apt-get install -y libeigen3-dev
RUN apt-get install -y libglfw3 libglfw3-dev 

# Needed for decompressing the camera model
RUN apt-get install -y liblz4-dev

# Install Eigen
RUN apt-get install -y libeigen3-dev

RUN apt -y remove x264 libx264-dev
 
## Install OpenCV 4 dependencies
RUN apt -y install build-essential checkinstall cmake pkg-config yasm
RUN apt -y install git gfortran
RUN apt -y install libjpeg8-dev libpng-dev

RUN apt -y install software-properties-common
RUN add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
RUN apt -y update

RUN apt -y install libjasper1
RUN apt -y install libtiff-dev

RUN apt -y install libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
RUN apt -y install libxine2-dev libv4l-dev

RUN apt -y install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
RUN apt -y install libgtk2.0-dev libtbb-dev qt5-default
RUN apt -y install libatlas-base-dev
RUN apt -y install libfaac-dev libmp3lame-dev libtheora-dev
RUN apt -y install libvorbis-dev libxvidcore-dev
RUN apt -y install libopencore-amrnb-dev libopencore-amrwb-dev
RUN apt -y install libavresample-dev
RUN apt -y install x264 v4l-utils
 
# Optional dependencies
RUN apt -y install libprotobuf-dev protobuf-compiler
RUN apt -y install libgoogle-glog-dev libgflags-dev
RUN apt -y install libgphoto2-dev libeigen3-dev libhdf5-dev doxygen
RUN apt -y install python3-dev python3-pip
RUN apt -y install python3-testresources
    
ENV LD_LIBRARY_PATH /mounted_libraries



