# Command to Execute
```
autolab up
autolab connect robot
sws
ros2 launch planning_bringup planning.launch.xml
```


Make sure that pointcloud, apriltags, etc is being published as ros topics thorough xavier, 
I use the script `pc3.py` to convert the data from websocket to ros topics [This script is in ~/coding/Autolab on Thor and is not a part of this PR/Code]

Publish one of these to `/planning_command` type `std_msgs::msg::String`
- plan_april
- plan_wellplate
- plan_home
- plan_home_offset
- idle

to command planning subsystem

The main logic of the code is in `robot/ros_ws/src/autonomy/5_planning/global_planner/src/move_to_pose_node.cpp`


Future improvements
- [Bug] *Currently, the code is not able to handle multiple April Tags
- [Design] Put all config params in one file
- [Design] Currently the main code is ~700 lines long, probably a good idea to split it to multiple chunks


