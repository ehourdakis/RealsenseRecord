#!/bin/bash
#Execute orb slam for TUM dataset
if [ -z "$1" ]
  then
    echo "No first argument supplied, first argument should be data directory"
    return 0;
fi

if [ -z "$2" ]
  then
    echo "No second argument supplied, second argument should ORBSLAM main directory"
    return 0;
fi

# Save current working directory
cwd=$(pwd)

# Goto to data dir
cd $1

$2/Examples/RGB-D/rgbd_tum $2/Vocabulary/ORBvoc.txt TUM.yaml ${PWD} associations_orb.txt

mv KeyFrameTrajectory.txt Results/KeyFrameTrajectory.txt
mv CameraTrajectory.txt Results/estimated_poses.csv

cd ${cwd}
