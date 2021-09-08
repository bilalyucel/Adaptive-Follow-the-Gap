# Camera-Based Mapping and Path Planning
### Task
- The main task of the project is to perform an obstacle avoidance algorithm while the vehicle is going to a given target point and map the places it passes during the trajectory. 
- For simulation purposes, Clearpath Husky Robot and Realsense D435 camera are chosen.

### Environment
- For simplicity Matlab/Simulink, ROS, Gazebo and Rviz tools are going to be used.
- For coding, Python is preffered.
- Some open-source packages are imported. (Will be explained in detail later)

### Obstacle Avoidance Algorithm
- There are many obstacle avoidance method exist in the literature however one recent noval approach "Follow the Gap" method is chosen because it is efficient and easy to use. For details, please check https://www.sciencedirect.com/science/article/abs/pii/S0921889012000838
- "Improved Follow the Gap" is also applied to the system for improvments. For details, please check https://ieeexplore.ieee.org/document/8014220
- For my project, an adaptive distance fuzzy logic controller is designed to improve the method because there were two missing critical points.
1. Fail of reaching the goal point when there is an obstacle near to it.
2. Early consideration of obstacles causes long trajectories.
- To solve these drawbacks, a fuzzy controlled adaptive consideration radius will be tuned based on 2 parameters that are distance to the goal and angle between nearest obstacle. As the vehicle moves to the target, consideration radius gets smaller thus even an obstacle exist near to the goal point, the vehicle will be able to reach it. Also, the algorithm considers the obstacle right in time so the vehicle will move in the hypotenuse resulting a shorter trajectory.

### Mapping
- As mentioned, vehicle would perform mapping as well with camera. There are several methods for mapping however "Octomapping" is chosen for our purposes. It will be used as a open-source package no code would be written for it.

### Requirements
- Install Ubuntu 16.04
- Install ROS Kinetic
- Install Clearpath Husky Robot
- Install Matlab Fuzzy Logic Toolbox

### How to run
- Clone this repository to your husky workspace(husky_ws). After you cloned, don't forget to make
```
catkin_make
```
- Add camera to the Husky robot (xacro file : https://www.clearpathrobotics.com/assets/guides/kinetic/husky/additional_sim_worlds.html)
```
export HUSKY_URDF_EXTRAS=$HOME/Desktop/realsense.urdf.xacro
```
- Spawn Husky Robot
```
cd husky_ws
source devel/setup.bash
roslaunch husky_gazebo husky_empty_world.launch
```
- Launch depthimage_to_laserscan node
```
roslaunch deptimage_to_laserscan dept_to_laser1.launch
```
- Run Octomapping
```
roslaunch octomap_server octomap_mapping.launch
```
- Run myMethod
```
rosrun scripts myMethod.py
```
- Run Rviz
```
rviz
```

### Demo
[![Graduation_Project](https://img.youtube.com/vi/wMMFzKAfvjo/0.jpg)](https://www.youtube.com/watch?v=wMMFzKAfvjo)
