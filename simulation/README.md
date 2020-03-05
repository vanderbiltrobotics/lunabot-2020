# Robot Simulation with Gazebo

The intent is to eventually be able to test a full integration of the software, so there is obviously a lot of work to do. 


##To Use

You must make sure that you have Gazebo 9.0 or higher.

If there is an error in the ros controller package, see https://discourse.ros.org/t/ros-control-abi-breakage-in-melodic-2020-02/12681 (some mofo broke compatibility). This means that certain other ROS packages need to be updated, but finding it is hard so most likely you will have to run the upgrade command (this will take a while).
```
sudo apt-get upgrade
```

Since we use a realsense sim package, after checking out the branch you will have to update a submodule.
```
git submodule update
```

##Launch Files

Launches Rviz (useful for checking out the robot description (URDF)
```
roslaunch simulation rviz.launch
```

Launches gazebo with the robot and a basic world (helper launch file)
```
roslaunch simulation gazebo_basic 
```

Launches the simulation such that joystick sends commands

```
roslaunch simulation gazebo_joystick.launch
```

Launches the simulation with the realsense, remapping args to get rtabmap to work correctly (this is why an object is spawned in front of the robot)

```
roslaunch simulation realsense_sim.launch
```



Note: No PID parameters are implemented for ros-control, so you will get an error message for both of these launches, but it is secretly fine as ros-control just doesn't use PID in this case.
