# ROS-Lab
Basic Robotic Operating System (ROS) lab tasks implemented in Python using ROS-Melodic.

### Environment
- Ubuntu 18.04LTS 64bit
- ROS-Melodic
- Python 2

### Labs
- Lab1: Square Drawing Robot in Gazebo
- Lab2: Obstacle Avoidance and Random Walk
- Lab3: Use Computer Vision to Move Towards Green Objects
- Lab4: Create Occupancy Grid Map
- Lab5: Coming Soon
  
### To Run
1. Clone this repository
```  
git clone https://github.com/joewong00/ROS-Lab.git
```

2. Create catkin workspace
```  
mkdir -p catkin_ws/src
```

3. Link the respective package to your workspace
```
sudo ln -fs /path_to_this_repo/lab1 ~/catkin_ws/src/lab1
```

4. ROS Setup
```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

5. Make _every_ python file executable
```
chmod a+x ~/catkin_ws/src/lab1/scripts/<YOUR_SCRIPT>.py
```

6. Launch python file package in respective lab
```
roslaunch lab1 lab1_closed.launch
roslaunch lab1 lab1_open.launch
```
  
### License
This project is licensed under the MIT License - see the LICENSE file for details.
If you want to cooperate or use this project, please contact : joewong00@gmail.com 
