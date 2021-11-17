# ROS-Lab
Basic ROS lab tasks implemented in Python using ROS-Melodic.

### Environment
- Ubuntu 18.04LTS 64bit
- ROS-Melodic
- Python 2

### Labs
- Lab1: Square Drawing Robot in Gazebo
- Lab2: Coming soon
- Lab3: Coming soon
- Lab4: Coming soon

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

5. Launch python file package in respective lab
```
roslaunch lab1 lab1_closed.launch
roslaunch lab1 lab1_open.launch
```
  

