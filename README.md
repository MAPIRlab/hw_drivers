# hw_drivers
A set of ROS pkgs implementing drivers to interact with sensors, actuators and mobile platforms.
  - giraff_ros_driver
  - giraff_interfaces
  - keyboard_control
  - [PR] [ROS2ARIA](https://github.com/MAPIRlab/ros2aria)
  - sickLMS
  - [PR] [Olfaction drivers](https://github.com/MAPIRlab/hw_drivers_olfaction)
  - [PR] [Motas](https://github.com/MAPIRlab/Motas)
  - [PR] urg_c
  - [PR] urg_node
  - [PR] urg_node_msgs

## Checkout
```
git clone --recursive https://github.com/MAPIRlab/hw_drivers.git
```
## Pull changes
If it's the first time you check-out a repo you need to use --init first:
```
git submodule update --init --recursive
```
For the next times:
```
git submodule update --recursive