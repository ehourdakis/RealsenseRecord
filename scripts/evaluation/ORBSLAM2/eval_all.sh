# If you want to mount the data
# sudo mkdir -p /media/manos/data/
# sudo mount -t ext4 /dev/sdb1 /media/manos/data/

#!/bin/bash
#Execute orb slam for TUM dataset
if [ -z "$1" ]
  then
    echo "No argument supplied, first argument should be data MAIN directory"
    return 0;
fi

# Save current working directory
cwd=$(pwd)

cd $1

for d in */ ; do
    [ -L "${d%/}" ] && continue
    #cls
    echo "Running slam on Dataset ${PWD}/$d"
    #source $REALSENSE_RECORD_DIR/scripts/evaluation/ORBSLAM2/run_remote2local_evaluation_pipeline.sh ${PWD}/$d /files/repos/orbslam2/ORB_SLAM2/ /home/manos/Desktop/Data/$d
    source $REALSENSE_RECORD_DIR/scripts/evaluation/ORBSLAM2/run_local_evaluation.sh ${PWD}/$d /files/repos/orbslam2/ORB_SLAM2/

    #Must go back to data directory
    cd $1
done    
