# Docker file for RealsenseRecorder
You can use this file to record data fast. After building it, the container can execute the RealsenseRecorder and dump data on a shared directory with the host.

## Installation and running
Build the docker image using the build script:
```bash
source build.sh
```
Run the image using the following command:
```bash
source run.sh felice/realsense:main
```
The above command will record 100 frames @ 30 FPS in the data directory of the host. 
## To distribute a new version perform the following steps
1. Drop the required OpenCV libraries into the libraries/ folder 
2. Update the rs_async_drop_RGBDIMU executable in the recorder folder
