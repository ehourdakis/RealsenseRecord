#!/bin/
# Converts a RealsenseRecord dataset to the TUM format.
# run as: 
# 
#!/bin/bash
# Convert RR data sequence to ORBSLAM2 format
# source convert_RR_to_TUM.sh ${DATA_DIR}
if [ -z "$1" ]
  then
    echo "No argument supplied, first argument should be data directory"
    return 0;
fi

# Save current working directory
cwd=$(pwd)

# Goto to data dir
cd $1

[ -d rgb_orb ]     || mkdir rgb_orb
[ -d depth_orb] || mkdir depth_orb

#cp rgb.txt rgb_ORIGINAL.txt
#cp depth.txt depth_ORIGINAL.txt

declare -i number=0
while read line; do 
	number=number+1
	#str=$(cat depth_aligned.txt | awk "{if(NR=="$number") print \$0}") # | awk {'print $2'})
	
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
	new_fname="rgb_orb/""$new_timestamp"".png"
	
	# new file line
	echo $prev_fname "will become ----> " $new_fname
	cp $prev_fname $new_fname
	echo $new_timestamp $new_fname >> rgb_orb.txt
done < rgb_aligned.txt


declare -i number=0
while read line; do 
	number=number+1
	#str=$(cat depth_aligned.txt | awk "{if(NR=="$number") print \$0}") # | awk {'print $2'})
	
	#Store the prev filename
	prev_fname=$(echo "$line" | awk {'print $2'})
	# Timestamp
	# Get the original timestamp 
	original_timestamp=$(echo "$line" | awk {'print $1'})

	# Divide /100 and keep the first 16 digits to convert to tum
	new_timestamp=$(echo "$original_timestamp / 1000.0" | bc -l | awk '{ print substr( $0, 0, 16 ) }')

	# new filename
	new_fname="depth_orb/""$new_timestamp"".png"
	
	# new file line
	echo $prev_fname "will become ----> " $new_fname
	cp $prev_fname $new_fname
	echo $new_timestamp $new_fname >> depth_orb.txt
done < depth_aligned.txt

cd ${cwd}