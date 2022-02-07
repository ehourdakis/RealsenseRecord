
# Script to execute the RealsenseRecorder docker image
# Expects the following parameters:
# 1. The name and tag of the image
# 2. The host directory to record the data in
# 3. The desired FPS to record the data (available options are 30, 60 and 90)
# 4. The number of frames to record
#
# Execute with the command:
# source run.sh felice/realsense:main "/home/data" 30 100

xhost +
docker run --runtime=nvidia -ti --rm -e DISPLAY -v${PWD}/data:/home/data  -v ${PWD}/libraries:/mounted_libraries -v ${PWD}/recorder/:/recorder -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix --privileged $1 /recorder/rs_async_drop_RGBDIMU $2 $3 $4
xhost -
