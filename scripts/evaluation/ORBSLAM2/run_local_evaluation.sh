
#!/bin/bash
#Execute orb slam for TUM dataset
#This script executes the orb slam evaluation for the fetched dataset, localy

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

# Save current working directory
cwd=$(pwd)
cd $REALSENSE_RECORD_DIR/scripts/evaluation/ORBSLAM2/

source change_intrinsics.sh $1
source run_orb2.sh $1 $2
#source evaluate.sh $1 -sa
source evaluate_one.sh $1

cd ${cwd}
