### ROS publisher for Realsense Record datasets
Use this package to publish Realsense Record datasets in ROS. Currently, the RGB and depth frames are supported.

#### Syncrhonizing data frames
If your data is not synchronized, you first must run the synchronization script by following these steps:

1. First make it an executable:
```
sudo chmod +x ${ROS_WORKSPACE}/realsense_record_publisher/scripts/assoc_rgbdi.py
```
2. And then execute it:
```
rosrun realsense_record_publisher assoc_rgbdi.py ${DATASET_DIRECTORY}
```
The script will generate the syncrhonized indexes with the _aligned prefix in the dataset directory.

#### Running the publisher
After the dataset is synchronized you can launch the publisher:
```
roslaunch realsense_record_publisher realsense_record_publisher.launch
```
The parameters of the publisher are located in the file ```launch/realsense_record_publisher.launch```.