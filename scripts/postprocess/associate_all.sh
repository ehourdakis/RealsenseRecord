#!/bin/bash

# Check if any arguments were passed
if [ $# -eq 0 ]; then
    echo "No arguments provided"
    echo "Usage: ./run_script.sh directory"
    exit 1
fi

# The first argument is the directory
dir=$1

# Loop through all first level subdirectories
for subdir in $dir/*/; do
    # cd into each directory
    cd $subdir

    # Echo the current directory
    echo "Processing $subdir"

    # Call your python script
    python /files/Projects/UnderDev/RealsenseRecord/scripts/postprocess/assoc_rgbdi.py depth.txt rgb.txt acc.txt gyr.txt

    # Go back to the original directory to start the next loop
    cd - > /dev/null
done

