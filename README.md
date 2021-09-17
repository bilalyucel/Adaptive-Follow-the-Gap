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
![goal_fail](https://user-images.githubusercontent.com/63864726/132513117-da7c7ca4-6ca8-4013-a896-8bcffaa6523a.jpg)
2. Early consideration of obstacles causes long trajectories.
![long_traj](https://user-images.githubusercontent.com/63864726/132513169-2166862e-a57d-416d-b0dd-0cfccab25597.jpg)
- To solve these drawbacks, a fuzzy controlled adaptive consideration radius will be tuned based on 2 parameters that are distance to the goal and angle between nearest obstacle. As the vehicle moves to the target, consideration radius gets smaller thus even an obstacle exist near to the goal point, the vehicle will be able to reach it. Also, the algorithm considers the obstacle right in time so the vehicle will move in the hypotenuse resulting a shorter trajectory.

<p align="center">
  <img src="https://user-images.githubusercontent.com/63864726/132514328-0c503d38-77b8-4071-8cd4-e8138da00f77.png" />
</p>

![Rule_table](https://user-images.githubusercontent.com/63864726/132514328-0c503d38-77b8-4071-8cd4-e8138da00f77.png)
![memberships](https://user-images.githubusercontent.com/63864726/132513388-9e630adf-79a6-415c-858e-8995cb4940db.jpg)
### Mapping
- As mentioned, vehicle would perform mapping as well with camera. There are several methods for mapping however "Octomapping" is chosen for our purposes. It will be used as a open-source package no code would be written for it.

### Requirements
- Install Ubuntu 16.04 (https://releases.ubuntu.com/16.04/)
- Install ROS Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu)
- Install Clearpath Husky Robot (https://github.com/husky/husky/tree/kinetic-devel)
- Install Matlab Fuzzy Logic Toolbox (https://www.mathworks.com/help/fuzzy/fuzzylogiccontroller.html)

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
### Results
Failure problem of reaching goal when a near obstacle exist around to target of FGM and FGM-I methods is resolved and overall efficiency is increased. The newly developed A-FGM approach adds a new parameter that is a consideration radius, to the previous methods. Now, the algorithm considers only obstacles within this radius which is tuned by a fuzzy logic controller that takes distance to the goal point and the angle of nearest obstacle as input and produces the consideration radius as output. With this modification, the vehicle can reach the goal point even if there is a near obstacle because the consideration radius gets smaller as the vehicle approaches the target. Also, the approach shortens the trajectory as it continues in the shortest path longer. These improvements are tested on possible cases to prove the algorithm.
![goal_suc](https://user-images.githubusercontent.com/63864726/132517794-7efa47a8-90c1-4a0d-a912-b4870a1245fe.jpg)
![long_traj_suc](https://user-images.githubusercontent.com/63864726/132517811-8e95299b-2d50-4b24-9eef-7a74495d13dd.jpg)


### Demo
[![Graduation_Project](https://img.youtube.com/vi/wMMFzKAfvjo/0.jpg)](https://www.youtube.com/watch?v=wMMFzKAfvjo)
