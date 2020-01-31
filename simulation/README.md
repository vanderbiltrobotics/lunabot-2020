# Robot Simulation with Gazebo

Hey guys, to use these files just make sure that you have Gazebo 9.0 or higher (you should if you got the default ROS melodic installation). The intent is to eventually be able to test a full integration of the software, so there is obviously a lot of work to do. 

TODO:
Integrate namespaces of joints (wheels) with teleop, add environment, and upgrade robot model once other teams are further ahead.

Launches the simulation with no controllers set up and an empty environment.

```
roslaunch simulation gazebo_sim.launch
```

Launches the simulation with ROS's differential drive controller, with a gui for contolling the velocity.

```
roslaunch simulation gazebo_sample_drive.launch
```

Note: No PID parameters are implemented, so you will get an error message for both of these launches, but it is secretly fine as ros_control just doesn't use PID in this case.
