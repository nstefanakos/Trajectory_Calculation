# Trajectory_Calculation
A script for calcuting the extrapolating trajectories of moving objects.

# Publishing Messages
1. A pose array that indicates the current location of the object and generates the extrapolated trajectories.
2. The implementation of a Ray Caster to delete any trajectories that are not feasible. (*Currently not done yet*)
3. An Occupancy Grid to generate a map of the enviroment. 

# Notes
- This project is designed with ROS nodes in mind.
- The LaserScan and Image msgs are stored in a rosbag that is not provided. You can change the path to your own rosbag by modifying the start.launch file.
