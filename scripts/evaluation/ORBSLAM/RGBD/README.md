## Instructions on how to execute ORBSLAM2 or ORBSLAM3 (without IMU) with the RealsenseRecord data, and evaluate the output using EVO
----
The directory contains a series of scripts that convert the RealsenseRecord data to the TUM ORBSLAM format, and evaluate it using EVO. 
In the following, by ```${RR_DATA_DIR}``` we mean the main RealsenseRecord data directory, while with ```${RR_DATA_CONVERTED_DIR}``` the converted directory. 
***
**Remember that you must set the $REALSENSE_RECORD environment variable to your RealsenseRecord installation. If you used the install script to install it, then this should be set in your ~/.bashrc file**
***

### Evaluate data without IMU (ORBSLAM2 and ORBSLAM3)
To run the evaluation pipeline, execute the following steps:
1. First execute the conversion script, to convert the RealseRecord data to the ORBSLAM TUM format. This will create two new folders in the given data dit, depth_orb and rgb_orb, along with two new indexes, rgb_orb.txt and depth_orb.txt. The command is: 

```bash
source convert_RR_to_TUM.sh ${RR_DATA_DIR} ${RR_DATA_CONVERTED_DIR}
```
2. Then you can run the run_orb2.sh script with the RR data directory, to execute ORBSLAM2.
```bash
source run_orb.sh ${RR_DATA_CONVERTED_DIR} ${ORBSLAM_DIR}
```
3. Finally, run the evaluation with EVO
```bash
source evaluate.sh ${RR_DATA_CONVERTED_DIR}
```
***
### One line evaluation
Alternatively you can evaluate one sequence with the command:
```bash
source evaluate_one_sequence.sh ${RR_DATA_DIR} ${ORBSLAM_DIR} ${RR_DATA_CONVERTED_DIR}
```
which will convert and evaluate on sequence of RealsenseRecord with the set orbslam.

If you have multiple data sequences in a directory, you can run:
```bash
source evaluate_all_sequences.sh ${RR_MAIN_DATA_DIR} ${ORBSLAM_DIR} ${RR_MAIN_DATA_CONVERTED_DIR}
```
where ```${RR_MAIN_DATA_DIR}``` is the main data directory containing the other RealsenseRecord data sequences as subdirectories. ```${RR_DATA_CONVERTED_DIR}``` is the main directory to hold the converted datasets. 

***
### Other directory

You can mass manipulate the results from a main data directory to another directory with the following scripts:
```bash
# Remove prev results
source do_in_dir_remove_results.sh ${RR_MAIN_DATA_DIR}
# Copy only results to directory
source do_in_dir_copy_results.sh ${RR_MAIN_DATA_DIR} ${COPYDIR}
``` 