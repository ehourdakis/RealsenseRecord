#!/bin/
# Converts a RealsenseRecord dataset to the TUM format.
# run as: 

# source convert_RR_to_TUM.sh ${RR_DATA_DIR} ${RR_DATA_CONVERTED_DIR}

if [ -z "$1" ];
  then
    echo "No argument supplied, first argument should be data directory"
    return 0;
fi
if [ -z "$2" ];
  then
    echo "No argument supplied, second argument should be the directory to store converted data"
    return 0;
fi

# Save current working directory
cwd=$(pwd)

# Goto to data dir
cd $1

echo "Creating directory structure"
# Make converted dir and create folders
[ -d $2 	      ]       || mkdir -p $2
[ -d $2/imu   	]     	|| mkdir -p $2/imu
[ -d $2/Results ]       || mkdir -p $2/Results

# Find the ground truth file and copy it as well
echo "Setting up files"
file_gt=$(find $1 -name 'gt.csv')
cp $file_gt $2/Results/gt.csv

# Copy the TUM.yaml and intrinsics
cp ${REALSENSE_RECORD_DIR}/scripts/evaluation/ORBSLAM/RGBDIMU/TUM.yaml $2/TUM.yaml
cp $1/rgb.intrisics $2/rgb.intrisics
# Put the intrinsics into TUM.yaml
source ${REALSENSE_RECORD_DIR}/scripts/evaluation/ORBSLAM/helpers/change_intrinsics.sh $2
rm $2/rgb.intrisics

# Copying raw data
echo "Copying the dataset to the new location"
echo "Copying IMU data"
cp -r $1/acc $2/acc
cp -r $1/gyr $2/gyr
echo "Copying RGB images"
cp -r $1/rgb $2/rgb
echo "Copying Depth images"
cp -r $1/depth $2/depth

cp $1/acc.txt $2/acc.txt
cp $1/gyr.txt $2/gyr.txt
cp $1/rgb.txt $2/rgb.txt
cp $1/depth.txt $2/depth.txt

# Rename the files and indexes in the new directory to have the ORBSLAM timestamp name 
# format, and create composite IMU files from acc and gyr
cd $2
echo "Creating the associations_orb.txt index"
python3 ${REALSENSE_RECORD_DIR}/scripts/evaluation/ORBSLAM/helpers/associate_rgbdi.py $2/depth.txt $2/rgb.txt $2/acc.txt $2/gyr.txt > $2/associations_orb.txt

cd ${cwd}