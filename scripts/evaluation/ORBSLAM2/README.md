# This is a set of instructions on how to convert and evaluate data acquired by RealsenseRecord with ORBSLAM2. 


1. First execute the convert_RR_to_TUM.sh script, to convert the RR data to ORBSLAM TUM format. This will create two new folders, depth_orb and rgb_orb, along with two new indexes, rgb_orb.txt and depth_orb.txt. The commant is: 

source convert_RR_to_TUM.sh ${RR_DATA_DIR}

2. Then run the associate script to create the associations_orb.txt index, for ORBSLAM2.

source associate_script.sh ${RR_DATA_DIR}

3. Make use you have TUM.yaml in the data directory, and setup with the camera intrinsics.

4. Run the run_orb2.sh script with the RR data directory, to execute ORBSLAM2.

source run.sh ${RR_DATA_DIR}

5. Run the evaluation with evo

source evaluate.sh ${RR_DATA_DIR} -sa