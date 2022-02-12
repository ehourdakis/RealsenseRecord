## Instructions on how to execute ORBSLAM2 with the RealsenseRecord data, and evaluate the output using EVO
----
The directory contains a series of scripts that convert the RealsenseRecord data to the TUM ORBSLAM2 format, and evaluate it using EVO. To run the evaluation pipeline, execute the following steps:
1. First execute the **convert_RR_to_TUM.sh** script, to convert the RealseRecord data to the ORBSLAM TUM format. This will create two new folders in the given data dit, depth_orb and rgb_orb, along with two new indexes, rgb_orb.txt and depth_orb.txt. The commant is: 

```bash
source convert_RR_to_TUM.sh ${RR_DATA_DIR}
```

2. Then run the associate script to create the associations_orb.txt index, for ORBSLAM2.

```bash
source associate_script.sh ${RR_DATA_DIR}
```
3. The above conversion process will dump the rgb.intrinsics of the dataset along with a unfiled TUM1.yaml file for orbslam. Make sure to fill it with:
```bash
source change_intrinsics.sh ${RR_DATA_DIR}
```
4. Then you can run the run_orb2.sh script with the RR data directory, to execute ORBSLAM2.
```bash
source run_orb2.sh ${RR_DATA_DIR} ${ORBSLAM2_DIR}
```
5. Finally, run the evaluation with EVO
```bash
source evaluate_one.sh ${RR_DATA_DIR}
```

If your data are in a remote folder, and you wish to convert and copy the orb data on a unique folder, use the **run_remote2local_evaluation_pipeline.sh** script. Otherwise, if you have your data already copied on a directory, use the **run_local_evaluation.sh** script. 

The script **eval_all.sh** will evaluate recursively, on all the data directories. Typical usage:
```bash
source eval_all.sh ${MAIN_DATASET_DIR}
```

Typical usage, if you have your data processed in the ${DIR} directory, then process them with ORBSLAM using the following commands:
```bash
# Remove prev results
source do_in_dir_remove_results.sh ${DIR}
# Evaluate with orbslam
source eval_all.sh ${DIR}
# Copy only results to directory
source do_in_dir_copy_results.sh ${DIR} ${COPYDIR}
``` 