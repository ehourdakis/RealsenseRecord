#!/bin/bash
if [ -z "$1" ]
  then
    echo "No argument supplied, first argument should be data directory that contains the rgb.txt and depth.txt indexes"
    return 0;
fi
# Usage:
# associate.sh /data/tum_rgbd_directory_data
# --max_difference 1000
python associate.py $1/rgb_orb.txt $1/depth_orb.txt > $1/associations_orb.txt
