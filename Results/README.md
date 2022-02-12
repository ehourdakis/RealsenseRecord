# **Evaluation of ORBSLAM2 using the Optitrack V120 Trio MoCap system**

The directory includes the results of the evaluation of the ORBSLAM2 algorithm using the Optitrack V120 Trio device. To evaluate the data we employed the scripts in the RealsenseRecorder **scripts/evaluation/ORBSLAM2** directory. The experimental setup is outlined in the following section.

## **Experimental Setup**

We use the Optitrack Mocap system to track the location of the robot and objects. It provides 6DoF pose measurements with 2mm accuracy at 120FPS. 

The robot used was a Xiaomi Mi robot vacuum, controlled with remote teleoperation, using a mobile phone. The picture below shows the setup of the Optitrack Trio, including the placement of the markers on the robot. 

<p align="middle">
  <img src="images/optitrack_rumba.JPG" width="400" />
  <img src="images/rumba_markers2.JPG" width="225" /> 
  <img src="images/tracking_objects.JPG" width="400" /> 
</p>
<p align = "center">
Fig.1 - (left) The Optitrack Trio setup and (middle) Xaomi robot, integrated with optical markers (right) object tracking using the optical markers.
</p>

For each sequence, we control the robot remotely. We use a Realsense D455 camera mounted on top of the robot to  acquire RGB, Depth, Accelerometer and Gyroscope measurements at different framerates. The dataset includes different scene setups, including scenes with a lot of features, scenes with very low feature density or reflective materials. Some example are shown below:
<p align="middle">
  <img src="images/low_features.JPG" width="300" />
  <img src="images/high_features.JPG" width="300" /> 
  <img src="images/reflective.JPG" width="300" /> 
</p>
<p align = "center">
Fig.2 - (left) Low feature sequence (middle) High feature sequence (right) reflective materials on floor.
</p>

We also change the lighting of the environment, to experiment with the tracking component of the VSLAM algorithm. The figures below show some example lighting conditions used:

<p align="middle">
  <img src="images/natural_light.JPG" width="350" />
  <img src="images/technical_light.JPG" width="350" /> 
</p>
<p align = "center">
Fig.3 - (left) some natural lighting (right) technical lighting only.
</p>

## **Moving Objects**
The dataset also includes sequences where objects move in the vicinity of the robot. We use the Optitrack Trio markers to track the pose of the object, relative to the Mocap frame. The figures below illustrate the setup:
<p align="middle">
  <img src="images/moving_objects_setup.JPG" width="280" />
  <img src="images/moving_objects.JPG" width="500" /> 
</p>
<p align = "center">
Fig.4 - (left) the setup for tracking the robot and object position (right) moving the object while controlling the robot.
</p>

The strings are used as a way to move the object while also controlling the robot :).
We use different colors and sizes for the objects, as shown below:

<p align="middle">
  <img src="images/moving_object_red.JPG" width="400" />
  <img src="images/moving_object_black.JPG" width="400" /> 
</p>
<p align = "center">
Fig.5 - (left) red small object (right) black medium object.
</p>

## **Evaluation**
We use the scripts in the **scripts/evaluation/ORBSLAM2** directory to evaluate the data sequences. Currently, there are two folders. The first includes data sequences with static scenes and is located in the **Results/Static** folder. These can be used as a benchmark for the VSLAM system. Some example evaluations are shown below:

<p align="middle">
  <img src="Static/H3U4J2I1E2A1L1F3T1M2R1K1/evo_traj_map.png" width="400" />
  <img src="Static/H3U4J2I1E2A1L1F3F5T1T7M1R1K2/evo_traj_map.png" width="400" /> 
<img src="Static/H3U4J2I1E2A1L1F3T1T2M1R1K2/evo_traj_map.png" width="400" /> 
</p> 
<p align = "center">
Fig.6 - The evaluation of different trajectories in static environments.
</p>

The second includes scenes with moving objects, which are more challenging for ORBSLAM2. In these sequences, the ground truth file also includes the XYZ position of the object in the scene. They are located in the **Results/MovingObjects** directory. Results are more messed up in here, and can definitely be improved. Some examples are below:

<p align="middle">
  <img src="MovingObjects/H3P2B3I1L1F2T1T7M1M3R1K1/evo_traj_map.png" width="400" />
  <img src="MovingObjects/H3P2B3I1L1F2T1T7M1R1K1/evo_traj_map.png" width="400" />  
<img src="MovingObjects/H3P2B3I1L1F2T1M4R1K1/evo_traj_map.png" width="400" /> 
</p> 
<p align = "center">
Fig.7 - The evaluation of different trajectories with moving objects.
</p>