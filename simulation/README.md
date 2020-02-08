# Robot Simulation with Gazebo

Hey guys, to use these files just make sure that you have Gazebo 9.0 or higher (you should if you got the default ROS melodic installation). The intent is to eventually be able to test a full integration of the software, so there is obviously a lot of work to do. 

TODO:
Integrate namespaces of joints (wheels) with teleop, add environment, and upgrade robot model once other teams are further ahead.

Launches the simulation with with joystick, to drive a robot around (woo)

```
roslaunch simulation gazebo_joystick.launch
```

Note: No PID parameters are implemented, so you will get an error message for both of these launches, but it is secretly fine as ros_control just doesn't use PID in this case.
