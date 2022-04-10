#!/bin/
# Converts a RealsenseRecord dataset to the TUM format.
# It will copy all data into a new converted directory, 
# and rename all indexes for rgb and depth to have the 
# timestamp in the filename. 
# It will also setup all settings and ground truth (if 
# any) files.

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
[ -d $2 	    ]     || mkdir -p $2
[ -d $2/rgb     ]     || mkdir -p $2/rgb
[ -d $2/depth   ]     || mkdir -p $2/depth
[ -d $2/Results ]     || mkdir -p $2/Results

echo "Copying files"
# Copy and rename the rgb files
declare -i number=0
while read line; do 
	number=number+1
	#Store the prev filename
	prev_fname=$(echo "$line" | awk {'print $2'})
	
	# Timestamp
	# Get the original timestamp 
	original_timestamp=$(echo "$line" | awk {'print $1'})
	# Remove first four digits from timestamp
	#timestamp=$(echo "$line" | awk {'print $1'}| awk '{ print substr ($0, 4 ) }' )
	#echo "$original_timestamp"
	
	# Divide /100 and keep the first 16 digits to convert to tum
	new_timestamp=$(echo "$original_timestamp / 1000.0" | bc -l | awk '{ print substr( $0, 0, 16 ) }')
	#Create the filename
	new_fname="rgb/""$new_timestamp"".png"
	
	# new file line
	echo $prev_fname "will become ----> " $new_fname
	cp $prev_fname $2/$new_fname
	echo $new_timestamp $new_fname >> $2/rgb.txt
done < rgb.txt

# Copy the depth files
declare -i number=0
while read line; do 
	number=number+1
	#Store the prev filename
	prev_fname=$(echo "$line" | awk {'print $2'})
	# Timestamp
	# Get the original timestamp 
	original_timestamp=$(echo "$line" | awk {'print $1'})

	# Divide /100 and keep the first 16 digits to convert to tum
	new_timestamp=$(echo "$original_timestamp / 1000.0" | bc -l | awk '{ print substr( $0, 0, 16 ) }')

	# new filename
	new_fname="depth/""$new_timestamp"".png"
	
	# new file line
	echo $prev_fname "will become ----> " $new_fname
	cp $prev_fname $2/$new_fname
	echo $new_timestamp $new_fname >> $2/depth.txt
done < depth.txt

# Find the ground truth file and copy it to the right folder
echo "Setting up files"
file_gt=$(find $1 -name 'gt.csv')
cp $file_gt $2/Results/gt.csv

# Copy the TUM.yaml and intrinsics
cp ${REALSENSE_RECORD_DIR}/scripts/evaluation/ORBSLAM/RGBD/TUM.yaml $2/TUM.yaml
cp $1/rgb.intrisics $2/rgb.intrisics
# Put the intrinsics into TUM.yaml
source ${REALSENSE_RECORD_DIR}/scripts/evaluation/ORBSLAM/helpers/change_intrinsics.sh $2
rm $2/rgb.intrisics

# associate.sh /data/tum_rgbd_directory_data
# --max_difference 1000
echo "Creating the associations_orb.txt index"
python ${REALSENSE_RECORD_DIR}/scripts/evaluation/ORBSLAM/helpers/associate_rgbd.py $2/rgb.txt $2/depth.txt > $2/associations_orb.txt

cd ${cwd}