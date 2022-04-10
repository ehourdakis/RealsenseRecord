#!/bin/bash
#Execute a command for all dirs
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
    echo "${PWD}/$d""Results/EVO"

    #Remove the results dir
    rm -rf "${PWD}/$d/Results/EVO/"

    rm "${PWD}/$d/Results/estimated_poses.csv"
    rm "${PWD}/$d/Results/KeyFrameTrajectory.txt"
    rm "${PWD}/$d/Results/synced_estimated_poses.csv"
    rm "${PWD}/$d/Results/synced_gt_tum.csv"
done    


cd ${cwd}