# ROS Tutorial

ROS sample services/nodes written in Rust.

* [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials/)
* [Catkin Tutorials](http://wiki.ros.org/catkin/Tutorials)

## Create custom package

Assuming existence of workspace **~/catkin_ws** ([See here](http://wiki.ros.org/catkin/Tutorials/CreatingPackage)):

```
cd ~/catkin_ws/src
```

then

```
catkin_create_pkg turtlesim
```

or 

```
catkin_create_pkg turtlesim std_msgs rospy roscpp
```

## Custom ROS packages

* turtlesim - custom (simplified) turtlesim. 
  * See [here](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv) 
* rosservices - ROS services explored
    * See [here](https://github.com/adnanademovic/rosrust/issues/145) and [here](https://gitlab.com/pmirabel/rosrust-boilerplate/-/tree/custom_srv/)

**IMPORTANT:** rosmsg_include macro works properly (when including ROS services) only with linux line end i.e. LF . It does now work properly (it will panic) with windows CR LF!!

## Running node remotely

On host running roscore (and robot node) configure (can be added into .bashrc):
```
export ROS_IP=192.168.1.115
export ROS_HOSTNAME=192.168.1.115
export ROS_MASTER_URI=http://localhost:11311
```

On host running remote node (e.g. control node) configure:

```
export ROS_IP=192.168.1.119
export ROS_HOSTNAME=192.168.1.119
export ROS_MASTER_URI=http://192.168.1.115:11311
```

## ROS camera

See [usb_cam](http://wiki.ros.org/usb_cam) package & **kamera** package in **res** folder.