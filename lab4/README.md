# Creating Occupancy Grid Map

- Occupancy grid map is created using basic robot odometry and laser scan
  - Initialize map with grid = -1
  - If obstacle detected at the end of the scan line, set grid = 1
  - If no obstacle detected in scan line, set grids along the scan line = 0.05
  
- Use teleop key control to drive the robot and explore around the house in order to get the full map
- Map is stored in 2D numpy array, which is displayed using opencv
- Extra features to be added:
  - Robot perform random walking and obstacle avoidance
  - Use movebase navigation to move the robot around the house