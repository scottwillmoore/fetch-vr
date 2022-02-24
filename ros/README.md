# ROS

ROS packages implemented to publish point clouds and other pose data for objects to be visualised in Unity. Our own ROS scripts are encapsulated in the fetch_vr package. This document will explain briefly how some of the scripts work and how to run them.

All nodes and topics are setup and ran using the launch files. Specifically, `fetch_vr.launch` will setup all required ROS functions. For details on topic names and optional parameters included arguments in some individual launch files, go read them. 

## Visualising point cloud
Objects on top of a table are captured as a point cloud:  

### 1. Transform the tf frame of the Fetch's coloured point cloud
Involved scripts (from out ROS package fetch_vr, unless otherwise specified):
```
- transform_point_cloud_publisher.py
- segment_point_cloud_publisher.py
- open3d_ros_helper.py
```

The coloured point cloud captured by the Fetch's head camera (from topic `/head_camera/depth_registed/points`) is specified in the `camera_rgb_optical_frame` tf frame. For simple visualisation in Unity and in Rviz, it is transformed to the frame in which the robot model is described, `base_link`. 

### 2. Process and clean up point cloud  
[This tutorial](https://towardsdatascience.com/how-to-automate-3d-point-cloud-segmentation-and-clustering-with-python-343c9039e4f5) was followed for much of this implementation.

Unecessary portions of the point cloud are segmented from the tabletop objects and removed. This included the surface of the table, backgrounds and floor. The table surface was identified as a plane with a large number of points, thus any object that have a large flat surface may be removed from the point cloud as well. To avoid this adjust the threshold in the script or use another object. Background and floors are specified by a distance threshold from the origin of the `base_link` frame, which can also be adjusted.

>An option is also included to compute bounding boxes around the objects, which can also be visualised in Unity. This was intended to provide a rough representation of the objects in case the point cloud itself was not clear enough. However, it was decided that the point cloud contained enough details. Additionally, as the entire object could not be captured by the stationary camera, the bounding boxes could not be oriented accurately to reflect the pose of the object (as the other side of the object away from the camera was not visible, thus the shape of the object could not be accurately determined). Computing the bounding boxes was also computationally costly and slowed down the refresh rate of the point cloud. This option is disabled by default, but just for the funs it can be turned on in the `point_cloud_publisher.launch` launch file, with the `compute_bounding_boxes` arg tag. 

## Aruco marker table detection
Involved scripts:
```
- marker_publisher *from package aruco_ros
- marker_transform_publisher
```
To model the table in Unity, its position is obtained from an Aruco marker stuck to the surface. The `aruco_ros` package is used to detect the marker's pose and ID, which are published to a topic. The transform from the robot's `base_link` frame to the marker's frame is also published, to help with recreating the pose of the table in Unity.

## Moveit planning scene setup
Involved scripts:
```
setup_planning_scene.py
```
While Moveit planning requests are sent and dealt with on the Unity end, the planning scene is set up in ROS, because it's easier. Collision objects are created in the position of the table, located by the aruco markers, as well as around the table to reflect the environment in the real world. The positioning is specified completely by hardcoding, so if you are using this you should probably rewrite it. 

## (Failed) External camera point cloud visualisation and combination with Fetch's point cloud
Functional scripts:
```
camera_transform_publisher.py
```
Ran using `external_camera_publisher.launch`, this does pretty much the same thing as the point cloud from the Fetch's camera, but with another individual camera. The one used was the ASUS Xtion Pro Live, connected to ROS using the `openni2_launch` package. Note that this device may not work with certain versions of USB ports, sources on the internet have conflicting experiences (like [here](https://answers.ros.org/question/249504/asus-xtion-live-pro-on-usb-30-ros-kinetic-ubuntu-1604/?answer=249524#post-id-249524)). For me USB 3.1 seemed to work but not USB 3.0. Do your own tests and searches. The launch file is to be ran on a computer connected to the external camera, with its ROS master set as the computer running the main launch file `fetch_vr.launch`.

Two options were tried to combine the point cloud from the external camera and the Fetch's head camera. The `point_cloud2_assembler` script from `laser_assembler` package is a quick way to do it, however their original colours are not preserved. This is most likely because the point cloud data is converted to the older `sensor_msgs/PointCloud` message type from the `sensor_msgs/PointCloud2` published by both cameras, before being assembed and converted back.   
I have also attempted to write my own script, `concatenate_clouds.cpp` which uses [pcl](https://pointclouds.org/)'s + operator to concatenate the point clouds at synchronised time frames, but it is a miserable mess and does not work.

The external camera uses the Aruco markers to find their own pose relative to the markers, and subsequently connect themselves to the robot's tf tree. The program will scream in agony about tf connections when the external camera does not see markers, but once it is able to detect them correctly it will function as inteded. However, even after the transform of the point cloud to the Fetch's `base_link` frame we still encountered a difference in the positions of the two point clouds. This is possibly due to poor camera calibration. We attempted to calibrate the Xtion following [this tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration), however the results was not satisfactory. The calibration of the IR camera also required the `calibrate_modify_contrast` package. The script `contrast_augmenter.cpp` needs to be running to process the ir image to be visible in the calibration tool (see [here](https://answers.ros.org/question/55524/asus-extrinsic-calibration-failing-on-ir-image/), the original package was modified to build with catkin).

## Setting up communication with Unity
Done using the `ros_tcp_endpoint` package, this is ran by the `tcp_endpoint.launch` launch file. The IP and port can be changed from the launch file, currently it is set to a port of 10000, and IP of 0.0.0.0 to broadcast to all machines connected to the same network. 

## Additional packages required:
These packages were installed on to the Fetch using apt:

openni2_launch  
aruco_ros  
aruco_msgs  
jsk_recognition_msgs  

And others:  
open3d(python3.6+ using pip)  
openCV(on the computer connected to the external camera)

*note: this is probably not an exaustive list. Try to build and install whatever the errors tell you.