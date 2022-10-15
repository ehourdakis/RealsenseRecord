### realsense_record_publisher
Use this package to publish Realsense Record datasets in ROS. Currently, the RGB and depth frames are supported.

#### Syncrhonizing data frames
If your data is not synchronized, you first must run the synchronization script by following these steps:

1. First make it executable:
```
chmod +x ${ROS_WORKSPACE}/src/realsense_record_publisher/scripts/assoc_rgbdi.py
```
2. And then run it:
```
rosrun realsense_record_publisher assoc_rgbdi.py ${DATASET_DIRECTORY}
```
The script will generate the syncrhonized indexes with the _aligned suffix in the dataset directory.

#### Running the publisher
After the dataset is synchronized, you can launch the publisher:
```
roslaunch realsense_record_publisher realsense_record_publisher.launch
```
The parameters of the publisher are located in the file ```launch/realsense_record_publisher.launch```.

#### Installation
To install the package, copy the realsense_record_publisher directory into your ROS workspace and build it.
Alternatively, you can create a symbolic link into the ROS WS
```
ln -s ROS/realsense_record_publisher ${ROS_WORKSPACE}/src/realsense_record_publisher
```
