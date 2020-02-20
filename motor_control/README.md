# Digging Code

Package for controlling the excavator.

The Pose2D message for the arm should be in meters and radians, and the origin is the base of the arm. X points towards the center of the robot, y points towards the sky, and theta measures the angle between the bucket and the y axis. 


Launches the excavation node along with rviz for visualization

```
roslaunch excavation rviz_dig.launch
```

Run the following set of commands to get ez-publisher for publishing arm_cmd's easily:

```
sudo apt-get install ros-melodic-rqt
sudo apt-get install ros-melodic-rqt-ez-publisher
rm ~/.config/ros.org/rqt_gui.ini
rosrun rqt_ez_publisher rqt_ez_publisher 
```
