# Camera-Based Mapping and Path Planning
### Task
- Main task of the project is to perform an obstacle avoidance algorithm while going to a given target point and map the places it sees. 
- For simulation purposes, Clearpath Husky Robot and Realsense D435 camera are chosen.

### Environment
- For simplicity Matlab/Simulink, ROS, Gazebo and Rviz tools are going to be used.
- For coding Python is preffered.
- Some open-source packages are imported.(Will be explained in detail later)

### Obstacle Avoidance Algorithm
- There are many obstacle avoidance method exist in the literature however one recent noval approach "Follow the Gap" method is chosen because it is efficient and easy to use. Also, it has developed with a fuzzy logic controller. For details, please check https://www.sciencedirect.com/science/article/abs/pii/S0921889012000838
- "Improved Follow the Gap" is also applied to the system for improvments. For details, please check https://ieeexplore.ieee.org/document/8014220
- ### An adaptive distance fuzzy logic controller is designed to improve the method because there was a missing critical point of it which can be explained as if there is an obstacle near to the goal point due to the method vehicle can never reach the target.

### Mapping
- As mentioned, vehicle would perform mapping as well with camera. There are several methods for mapping however "Octomapping" is chosen for our purposes. It will be used as a open-source package no code would be written for it.

### Requirements
- Install Ubuntu 16.04
- Install ROS Kinetic
- Install Clearpath Husky Robot
- Install Matlab Fuzzy Logic Toolbox

### How to run
- Spawn Husky Robot```
cd husky_ws
roslaunch husky_gazebo husky_empty_world.launch
- Launch depthimage_to_laserscan node```
roslaunch deptimage_to_laserscan dept_to_laser1.launch
- Run myMethod```
rosrun scripts myMethod.py```
### Demo
[![Graduation_Project](https://img.youtube.com/vi/wMMFzKAfvjo/0.jpg)](https://www.youtube.com/watch?v=wMMFzKAfvjo)
