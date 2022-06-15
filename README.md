# P-AgBot: In-Row & Under-Canopy Agricultural Robot for Monitoring and Physical Sampling

This repository contains a code for crop monitoring modules (corn height, corn stalk radius) for ROS compatible UGVs. For corn height estimation, 3D point cloud from a vertically mounted 3D LiDAR Ouster OS1-64 are used. A 2D laser scan from a inexpensive 2D LiDAR ROBOTIS LDS-01 is used to estimate each corn stalk radius. The result of each module of our system can be found here. -> https://youtu.be/CU-CX41oVyk

<p align='center'>
    <img src="/pagbot_demo.gif" alt="drawing" width="800"/>
</p>

## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (Tested with melodic.)
- [Clearpath Jackal simulation](https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/simulation.html) (Follow the instructions to use the simulation.)

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/kimkt0408/P-AgBot.git
cd ..
catkin_make
```

## The system

P-AgBot consists of a unique combination and configuration of integrated sensors. P-AgBot and its components are as follows: (1) Tracking camera, (2) 3D LiDAR sensor, (3) Two-finger style gripper, (4) RGB-D camera, (5) Six degree-of-freedom robotic arm, (6) Servo motor, (7) 3D printed linkage with nichrome wire end-effector, and (8) 2D LiDAR sensor.

<p align='center'>
    <img src="/p_agbot_description.png" alt="drawing" width="400"/>
</p>

## Run the package

1. Run the simulation world launch file:
```
roslaunch jackal_simulation_iros.launch
```

2. Run the localization launch file:
```
roslaunch amcl_lds.launch
```

3. Run the navigation launch file:
```
roslaunch autonomous_navigation.launch
```

4. (1) Run the crop height estimation launch file:
```
roslaunch crop_height.launch
```

4. (2) Run the stalk radius estimation launch file:
```
roslaunch stalk_radius.launch
```



