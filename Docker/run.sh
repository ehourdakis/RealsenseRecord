
# Execute the given docker image
# Expects on parameter the name of the container
#
# ./run.sh felice/realsense:main
xhost +
docker run --runtime=nvidia -ti --rm -e DISPLAY -v${PWD}/data:/home/data  -v ${PWD}/libraries:/mounted_libraries -v ${PWD}/recorder/:/recorder -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix --privileged $1 /recorder/rs_async_drop_RGBDIMU /home/data 60 100
xhost -
