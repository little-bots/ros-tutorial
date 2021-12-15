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

**IMPROTANT:** rosmsg_include macro works properly (when including ROS services) only with linux line end i.e. LF . It does now work properly (it will panic) with windows CR LF!!