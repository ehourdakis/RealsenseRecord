# FORTH RealsenseRecord

[![Build Status](https://travis-ci.org/anfederico/Clairvoyant.svg?branch=master)](https://travis-ci.org/anfederico/Clairvoyant)
![Dependencies](https://img.shields.io/badge/dependencies-up%20to%20date-brightgreen.svg)

## About
The RealsenseRecord project provides tools for recording and synchronizing the RGB, Depth, Gyroscope and Accelerometer measurements from realsense cameras.
## Prequisites and installation

```bash
// Clone the repository
git clone http://git.ehourdakis.com/RealsenseRecord

// Configure and build the project
cd RealsenseRecord
mkdir build && cd build
cmake ..

make -j4
```

This process will compile two different executables, described  below:
| Application | Description  |
| ------ | -----------  |
| rs_pipeline_group_frame  | Retrieve RGB and Depth frames using the framesets posted by the device, and store them as groups of RGB, Depth, and multiple paired accelerometer and gyroscope measurements |
| rs_async_drop_RGBDIMU  | Drop the data frames from the device callback directly to memory, and index them based on their timestamp.  | 

## Notes on timestamping and Intel realsense cameras
During the implementation I have stumbled upon numerous posts commenting on the timestamping mechanism used for the IMU frames, by the realsense pipeline. A more detailed discussion on retrieving the IMU frames can be found [here](https://www.intelrealsense.com/how-to-getting-imu-data-from-d435i-and-t265/). In summary, IMU frames are timestamped according the rule below:

*Each IMU data packet is timestamped using the depth sensor hardware clock to allow temporal synchronization between gyro, accel and depth frames.*

The SDK provides different methods for timestamping. To enable it, set the CMake customization flag ENFORCE_METADATA to true when building the realsense sdk (it is set to False by default).
## Calibrating the IMU 
The IMUs of the D435i and D400 cameras do not include any internal callibration. To calibrate these devices, Intel has made available the following tool [link](https://github.com/IntelRealSense/librealsense/tree/development/tools/rs-imu-calibration#rs-imu-calibration-tool), which will calibrates and stores the IMU intrisics on device's NVRAM for later use in SDK. 

## Advanced Camera configuration
Within the source code there are excerts that access and change the properties of the different sensor devices on the camera, including disabling the auto-exposure filter or setting it to a different value.

## Auto-exposure filter and sensor frame rate
The autoexposure filter used by the device can significanlty affect the acquisition rates of the underlying RGB and depth sensors (you can read [here](https://github.com/IntelRealSense/librealsense/issues/4480#issuecomment-514055336) for more ifno). There have been examples where either the depth or RGB streams acquire data with a very reduced framerate, due to the delays imposed by the autoexposure filters. Due to this, it is always recommended to check the number of RGB and Depth frames recorded, and make sure that they are approximately the same size. If there are high inconsistencies, you can try changing the default auto-exposure values of the two sensors, to match their framerates. 
One -rather generic- good advice is to sample record your data when there is abundant natural lighting on the scene. 