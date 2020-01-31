# Digging Code

Package for controlling the excavator.


Launches the excavation node along with rviz for visualization
```
roslaunch robot_digging rviz_dig.launch
```

Run the following set of commands to get ez_publisher for publishing arm_cmd's easily:

```
sudo apt-get install ros-melodic-rqt
sudo apt-get install ros-melodic-rqt-ez-publisher
rm ~/.config/ros.org/rqt_gui.ini
rosrun rqt_ez_publisher rqt_ez_publisher 
```