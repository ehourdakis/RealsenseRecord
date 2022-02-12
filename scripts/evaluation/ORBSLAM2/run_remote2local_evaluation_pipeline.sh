#!/bin/bash
#Execute orb slam for TUM dataset
#This fetches the data from the hard drive first, converts to orb format, and then runs evaluations. 

if [ -z "$1" ]
  then
    echo "No argument supplied, first argument should be data directory"
    return 0;
fi

if [ -z "$2" ]
  then
    echo "No second argument supplied, second argument should ORBSLAM2 main directory"
    return 0;
fi
if [ -z "$3" ]
  then
    echo "No third argument supplied, third argument should where to store the converted data"
    return 0;
fi
# Save current working directory
cwd=$(pwd)
cd $REALSENSE_RECORD_DIR/scripts/evaluation/ORBSLAM2/

source convert_RR_to_TUM.sh $1 $3
source associate_script.sh $3
source run_orb2.sh $3 $2
source evaluate.sh $3 -sa

cd ${cwd}
