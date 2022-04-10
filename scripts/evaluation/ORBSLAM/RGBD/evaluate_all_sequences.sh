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
if [ -z "$2" ]
  then
    echo "No second argument supplied, second argument should ORBSLAM main directory"
    return 0;
fi
if [ -z "$3" ]
  then
    echo "No third argument supplied, third argument should be the directory to store the converted dirs."  
fi

# Save current working directory
cwd=$(pwd)

cd $1

for d in */ ; do
    [ -L "${d%/}" ] && continue
    #cls
    echo "Running slam on dataset: ${PWD}/$d and saving on $3/$d"
    
    # # Create the converted dir name by adding the prefix
    # converted_dir="${PWD}/${d%/}$prefix";
    # echo "Converted dir: $converted_dir"

    converted_dir="$3/$d"

    source ${REALSENSE_RECORD_DIR}/scripts/evaluation/ORBSLAM/RGBD/evaluate_one_sequence.sh ${PWD}/$d $2 $converted_dir

    #Must go back to data directory
    cd $1
done

cd ${cwd}