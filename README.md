# pyo_ros

This project aim to control object like _turtlesim_ virtualized or P3AT more real with the simple move of the arm because it's more intuitive than an rc control. The object needed are a Myo armband and a P3AT.
The fact that Myo has IMU sensors has inducted me to develop two disctinct way of control the vehicle, first way is with the EMG sensor and second one with the IMU sensor.

## Requirements:
  * [ROS](https://wiki.ros.org/lunar/Installation/Ubuntu)
  * [catkin](https://wiki.ros.org/catkin)
  * python2.7
  * [ROSAria](https://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA)
  
## How to install:
  * _cd catkin_ws_
  * _source devel/setup.bash_
  * _cd src_
  * _git clone https://github.com/SirSimonSays/pyo_ros

## How to launch:
  * turtlesim controlled by Myo
    * _roslaunch pyo_ros turtlePOSEdriver.launch_
    * _roslaunch pyo_ros turtleIMUdriver.launch_
  * P3AT controlled by Myo
    1. connect P3AT with serial bus
    1. first terminal: _roscore_
    1. second terminal: _rosrun rosaria RosAria_
    1. third terminal: _python scripts/turtleTalkerIMU.py_ or _python scripts/turtleTalkerPOSE.py_
    1. fourth terminal: _python scripts/keyboard.py_
  
## Useful link:
  * https://wiki.ros.org/ROS/Tutorials
