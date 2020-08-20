# Sensor Fusion Module
This module integrates the Velodyne Puck 16 and a camera sensor in order to enhance pedestrian detecion.

## Requirements
* python-pcl
* Gazebo 8+


## How to Run
```
    catkin_make (gem_ws)
    source devel/setup.bash (gem_ws)
    roslaunch gemulator sensorFusion.launch (in any folder)
    python mp2.py (gem_ws/src/gemulator/src/sensorFusion)
```

## File Descriptions
* mp2.py - Run this to deploy the whole system
* controller.py - contains controller and dynamic model of GEM
* sensorFusion.py - Fuses LiDAR and Camera Data
* dataCollection.py - Run this for static data collection (outputs point cloud, raw camera image, and pedestrian bounding boxes)
* lidarProcessing.py - Clusters point cloud data from the Velodyne LiDAR
* videoProcessing.py - Processes image from camera to determine if an pedestrian exists



## Overview


