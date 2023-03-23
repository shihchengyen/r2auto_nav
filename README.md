# r2auto_nav
ROS2 Code for EG2310, AY22/23 Sem2

This is Group 11's repository for EG2310. Here we attempt to create a turtlebot that will navigate through a restaurant to deliver a can it receives from a dispenser to a certain table.

 
## System Test
Run the following commands in different terminals to test the turtlebot.

### //TODO
  //TODO
### //TODO
  //TODO
### //TODO
  //TODO

## Launching the robot

Run the following commands in different terminals
### Turtlebot Bringup
```
  ssh ubuntu@(ip-address-of-pi)
  roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
### Run Cartographer
```
  ros2 launch turtlebot3_cartographer cartographer.launch.py
```
### Run Map2Base publisher
```
  ros2 run auto_nav map2base
```
### Control the turtlebot to store waypoints
```
  ros2 run auto_nav r2waypoints
```
### Run Master Script
```
  ros2 run auto_nav master_nav
```


  


