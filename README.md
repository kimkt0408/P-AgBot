# P-AgBot: In-Row & Under-Canopy Agricultural Robot for Monitoring and Physical Sampling

This repository contains a code for crop monitoring modules (corn height, corn stalk radius) for ROS compatible UGVs. For corn height estimation, 3D point cloud from a vertically mounted 3D LiDAR Ouster OS1-64 are used. A 2D laser scan from a inexpensive 2D LiDAR ROBOTIS LDS-01 is used to estimate each corn stalk radius. A demonstration of the system can be found here.

## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with melodic)

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git
cd ..
catkin_make
```
