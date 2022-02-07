# Docker file for RealsenseRecorder
You can use this file to record data directly, by running the pre-compiled application within a Docker container. 

Simply build and execute the provided Docker container, and specify where to dump the recorded data.

## Installation and usage
Build the docker image using the build.sh script:
```bash
source build.sh
```
Run the image using the following command:
```bash
source run.sh felice/realsense:main "/home/data" 30 100
```
The above command will record 100 frames at 30 FPS in the "/home/data" directory.
## To distribute a new version:
* Drop the required pre-compiled libraries (e.g. OpenCV) into the libraries/ folder 
* Update the rs_async_drop_RGBDIMU executable in the recorder folder
