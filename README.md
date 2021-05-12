# Camera-Based Mapping and Path Planning
### Task
- Main task of the project is to perform an obstacle avoidance algorithm while going to a given target point and map the places it sees. 
- For simulation purposes, Clearpath Husky Robot and Realsense D435 camera are chosen.

### Environment
- For simplicity ROS, Gazebo and Rviz tools are going to be used.
- For coding Python is preffered.
- Some open-source packages are imported.(Will be explained in detail later)

### Obstacle Avoidance Algorithm
- There are many obstacle avoidance method exist in the literature however one recent noval approach "Follow the Gap" method is chosen because it is efficient and easy to use. Also, it has developed with a fuzzy logic controller. For details, please check https://www.sciencedirect.com/science/article/abs/pii/S0921889012000838
- 
