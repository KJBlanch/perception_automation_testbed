# Project 'Beach' - Robotic Foot Placement. 

### Version & Changelog
Current - 0.2

#### 0.0.2 
Migrated to Best Practices. Seperated .h/.cpp and increased cohesion and reduced coupling. 

#### 0.0.1
Able to process the best point in multiple pointclouds, and output a single xyz value that would suit the current leg. 

### Author

Krister Blanch
 - kris.blanch@griffithuni.edu.au

### Licenses

BSD

### Dependencies 

These are required.

 - Ubuntu (18.04.3 LTS tested)
 - ROS Melodic (1.14.3 tested)
 - Point Cloud Libraries (PCL) for ROS. 

These are packaged. Listed here in case of accidental file deletion.

 - plane_seg - Authored by Krister Blanch.

### Status

Current Build maintained till December 13th, 2019. 

### About

This system is specific to a multi-legged robotic platform. The system takes in multiple 3d point clouds, performs planar segmentation and seperation, determines which plane is closest to a target location and then broadcasts the meanpoint of that plane to to a published node. 

As the robot takes steps, each leg is sent to this system to determine the range of the foot. If a plane falls outside this range, it is discarded. This saves computational time. Planes that fall partially within and without the range are cut accordingly. 

### Install

1. Decompress the .tar, .zip or whatever other file format I decided to package this in. 

 - You should see 'build, devel, install, src and this README'. 
 - If the build isn't 100% usable from decompression, delete the 'build, devel, install' folders and open a terminal inside the parent folder. Type the following;

 - catkin build

### Use

#### Standalone

##### Case 1. (Live feed using the Royale_in_Ros drivers)

1. Open a terminal.

2. Type 'roscore' (without quotation marks)

3. Open another terminal in the unzipped folder.(You'll need a few of these. In the event that a Real-Time process is run in a terminal, (i.e, ROSCORE), then start a new terminal and run the next step again. If a step has 'RTS' at the end of the line, it's a Real-time process'

4. type 'source devel/setup.bash' (without quotation marks)
5. Navigate to the Royale system, and start it. Typically by typing 'roslaunch royale_in_ros camera_driver.launch' (without quotation marks) RTS
6. type 'roslaunch plane_seg plane_seg.launch' (without quotation marks) RTS

 - This should start the system. 

7. You can then look at a visual representation of the pointcloud by opening up an RVIZ (new terminal, type 'rviz' and setting the paramaters to look at the 'planes' stream. 
8. To see the optimal foot placement, add a marker and set it to look for the 'visualization_marker'

##### Case 2. (Bag file using the AL leg or a bag file). CURRENT BUILD.

1. Open a terminal.

2. Type 'roscore' (without quotation marks)

3. Open another terminal in the unzipped folder.(You'll need a few of these. In the event that a Real-Time process is run in a terminal, (i.e, ROSCORE), then start a new terminal and run the next step again. If a step has 'RTS' at the end of the line, it's a Real-time process'

4. Launch the bag file. Terminal 'rosbag play *filename.bag*' (without quotation marks) RTS

5. type 'source devel/setup.bash' (without quotation marks)
6. type 'roslaunch plane_seg plane_seg.launch' (without quotaiton marks) RTS

 - This should start the system. 

7. You can then look at a visual representation of the pointcloud by opening up an RVIZ (new terminal, type 'rviz' and setting the paramaters to look at the 'planes' stream. 
8. To see the optimal foot placement, add a marker and set it to look for the 'visualization_marker'



### Bugs

 - When editing the workspace, make sure the time/date settings are correct. ROS has some weird compiling bugs due to time-travel non-compliance. 
 - Still testing the optimal number of points to seperate the planes. 
