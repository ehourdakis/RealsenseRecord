# FORTH RealsenseRecord

[![Build Status](https://travis-ci.org/anfederico/Clairvoyant.svg?branch=master)](https://travis-ci.org/anfederico/Clairvoyant)
![Dependencies](https://img.shields.io/badge/dependencies-up%20to%20date-brightgreen.svg)

## About
The RealsenseRecord project provides tools for recording and synchronizing the RGB, Depth, Gyroscope and Accelerometer measurements from realsense cameras.
## Prequisites and installation
Use the following steps to compile the project:
```bash
# Clone the repository
git clone http://git.ehourdakis.com/RealsenseRecord
```
Execute the installation scripts in the **scipts/installation** directory to install the project's dependencies. Then configure and compile it as shown below: 
```bash
# Configure and build the project
cd RealsenseRecord/
mkdir build && cd build

# Configure cmake by specifying the directories of your local opencv4 and realsense SDK installation
cmake -DOpenCV_DIR:STRING="$OPENCV_INSTALL"/lib/cmake/opencv4/          \
      -Drealsense2_DIR:STRING="$REALSENSE_INSTALL"/lib/cmake/realsense2 ..

# Compile 
make -j4
```
This process will compile two different executables, described  below:
| Application | Description  |
| ------ | -----------  |
| rs_pipeline_group_frame  | Retrieve RGB and Depth frames using the framesets posted by the device, and store them as groups of RGB, Depth, and multiple paired accelerometer and gyroscope measurements. |
| rs_async_drop_RGBDIMU  | Drop the data frames from the device callback directly to memory, and index them based on the HW timestamp. | 

To record a dataset, run the following command:
```bash
# Record 300 frames in the home/datadir/ directory
./rs_async_drop_RGBDIMU "/home/datadir/" 300 
```
All data will be stored in directory specified in the first argument. This has the following structure:
```
datadir
├── rgb
│   ├── **/*.png
├── depth
│   ├── **/*.png
├── acc
│   ├── **/*.txt
├── gyr
│   ├── **/*.txt
├── rgb.txt
├── depth.txt
├── imu.txt
├── rgb.intrisics
```
The rgb and depth directories hold the corresponding RGB/ and Depth/ files in a .png format. The acc/ and gyr/ directories hold the accelerometer and gyroscope measurements as a 3D vector stored in a .txt file. The files are indexed by order of first appearance. The files rgb.txt, depth.txt, acc.txt and gyr.txt store the timestamp and the index of each image. Finally, the file rgb.intrisics holds the intrinsics of the RGB device, as a 3x3 matrix stored in a text format. 
## Syncrhonization
The aforementioned data are indexed directly as obtained by the device. However, the different device streams in the realsense cameras are not syncrhonized. For example, it has been reported that there is a consistent lag between the IMU and RGB-D measurements. To compensate for this we must post-process the data, in order to associate the different data frames, based on their timestamps. The directory scripts/postprocess includes scripts to make this alignemnt. 
```bash
# Align the timestsamps
python scripts/postprocess/assoc_rgbdi.py rgb.txt depth.txt acc.txt gyr.txt
```
Which will generate the following new file indexes:
```
├── rgb_aligned.txt
├── depth_aligned.txt
├── acc.txt
├── gyr.txt
```
While recording, the only computational intensive operation that takes place is aligning the RGB image to the Depth image. You can try adding additional data filters, however, adding too many will affect the frame rate of the camera. All images are indexed based on their timestamp on a map, and saved to the database format after the acquisition process ends.

## Intel realsense cameras and timestamps
The realsense [documentation](https://intelrealsense.github.io/librealsense/doxygen/classrs2_1_1frame.html) describes four different types of timestamps:
| Timestamp Type | Description  |
| ------ | -----------  |
|SENSOR_TIMESTAMP | Device clock. For video sensors designates the middle of exposure.  |
|FRAME_TIMESTAMP | Device clock. Stamped at the beginning of frame readout and transfer. |
|BACKEND_TIMESTAMP | Host (EPOCH) clock in Kernel space. Frame transfer from USB Controller to the USB Driver. |
|TIME_OF_ARRIVAL | Host (EPOCH) clock in User space. Frame transfer from the USB Driver to Librealsense.|

To enable the first two types set the CMake flag ENFORCE_METADATA to true when building the realsense sdk (it is set to False by default).

Below is the default behavior on timestamping, described in the documentation:

*The function call provides the TIME_OF_ARRIVAL stamp. In case the metadata is available the function returns:
HW Timestamp (FRAME_TIMESTAMP), or
Global Timestamp Host-corrected derivative of HW Timestamp required for multi-sensor/device synchronization
The user can select between the unmodified and the host-calculated Hardware Timestamp by toggling the RS2_OPTION_GLOBAL_TIME_ENABLED option.*

A more detailed discussion on retrieving the IMU frames can be found [here](https://www.intelrealsense.com/how-to-getting-imu-data-from-d435i-and-t265/). In summary, IMU frames are timestamped according the rule below:

*Each IMU data packet is timestamped using the depth sensor hardware clock to allow temporal synchronization between gyro, accel and depth frames.*

## Calibrating the IMU 
The IMUs of the D435i and D400 cameras do not include any internal callibration for the IMU device. To calibrate these devices, Intel has made available the following [tool](https://github.com/IntelRealSense/librealsense/tree/development/tools/rs-imu-calibration#rs-imu-calibration-tool), which can be used to calibrate and store the IMU intrisics on the device, so that the SDK can retrieve them.  
## Evaluating datasets
The directory scripts/evaluation/ includes scripts to evaluate the output of a vSLAM algorithm against some ground truth, using the **EVO** software. You can download it [here](https://github.com/MichaelGrupp/evo). In short, you can install it by running:
```bash
pip install evo --upgrade --no-binary evo
```

Below is a description of each script in included:


| Script name | Description  |
| ------ | -----------  |
|align_data.m | Use matlab to align the timeseries of two external devices on the software. This can be used to align the ground truth measurements from the mocap system with the camera measurements |
|eval_evo.sh | Evaluate the output of a vSLAM trajectory against a Ground Truth file using EVO. This will evaluate the absolute pose error and relative pose error for the translation and rotation components invividually. |
|quick_eval.sh | Evaluate and store the data on a .zip file (useful for batch processing). |

A typical command to evaluate a dataset would be:
```bash
./eval_evo.sh 1 1 "datadir/" -sa >> Results/evo_out.txt 
```
## Advanced Camera configuration
If supported by the camera, one can load an advanced configuration of the different sensor devices on the camera, stored as a .json format. The directory configs/ contains an example .json. One good practice is to use the realsense-viewer in order to test different filter configurations, and then save the corresponding .json. 

## Auto-exposure filter and sensor frame rate
The autoexposure filter used by the device can significanlty affect the acquisition rates of the underlying RGB and depth sensors (you can read [here](https://github.com/IntelRealSense/librealsense/issues/4480#issuecomment-514055336) for more info). There have been examples where either the depth or RGB streams acquire data with a very reduced framerate, due to the delays imposed by the autoexposure filters. If there are high inconsistencies between the RGB and depth data acquisition rates, you can try changing the default auto-exposure values of the two sensors.
When possible, record your data when there is abundant natural lighting on the scene. 
## Other useful stuff
HW synchronization of multiple cameras is described in this [white paper](https://dev.intelrealsense.com/docs/external-synchronization-of-intel-realsense-depth-cameras?_ga=2.163875643.506586051.1643829742-1302374545.1643264215).