## Notes on running mapping

1. Check Journal Entry 17 to verify proper installation of OpenNI and RtabMap.
2. Run openni with the following:
```
roslaunch mapping openni.launch depth_registration:=true
```
3. Run rtabmap with the following:
```
roslaunch mapping openni_mapping.launch depth_registration:=true rtabmap_args:="--delete_db_on_start"
```

