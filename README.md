# r2auto_nav
ROS2 Code for EG2310, AY22/23 Sem2

This is Group 11's repository for EG2310. Here we attempt to modify a TurtleBot3 to navigate through a restaurant to deliver a can it receives from a dispenser to a certain table.


## Prerequisites

### Access to the following
1. Have access to a computer with Ubuntu 20.04 installed. A dual boot Linux is recommended but a virtual machine will suffice too. 
1.1 For those with M series Macbook, you can follow [this tutorial](https://www.youtube.com/watch?v=suntoEurFio) to install Ubuntu onto your mac. Make sure to allocate 40GB. 35GB will barely scrape through.
2. Have access to a TurtleBot3 kit. 
2.1 Have the ability to ssh into the TurtleBot3 RPi.
3. Have access to an ESP32.

### Make sure you have done the following before proceeding with the testing and the mission.
1. Follow [the TurtleBot3 manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) **(Make sure to select Foxy.)** to install Ros2 and other relevant packages onto your linux computer. You can stop once you are able to teleoperate the turtlebot from your computer.
2. Follow [this guide](https://ask.wingware.com/question/3/i2c-problem-with-remote-raspberry-pi/) to allow RPi I2C. 
3. //TODO follow guide to install MQTT



## System Test
Run the following commands in different terminals to test the turtlebot.

### //TODO
  //TODO
### //TODO
  //TODO
### //TODO
  //TODO

## Start the mission 
ENSURE that the ip address in the ESP32 and TableNumber.py is what is displayed by the broker in terminal after running.
```
hostname -I.
```
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
### Run TableNumber publisher
```
  ros2 run auto_nav tableNumber
```
### Control the turtlebot to store waypoints
```
  ros2 run auto_nav r2waypoints
```
### Run Master Script
```
  ros2 run auto_nav master_nav
```


  


