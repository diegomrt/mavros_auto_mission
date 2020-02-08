# mavros_mission_px4 #

ROS package (tested in ROS Kinetic) for sending a mission via MavLINK to a PX4-based aerial vehicle (real or simulated), remove failsafes, arm the motors, enter Mission mode and complete the mission.

Test with a simulated fixed-wing aircraft in this video:

[![Video](https://img.youtube.com/vi/jDYtM7kgN5o/maxresdefault.jpg)](https://youtu.be/jDYtM7kgN5o)


## Dependencies ## 

Besides ROS, both [MavROS](http://wiki.ros.org/mavros) and [PX4 Firmware](https://dev.px4.io/v1.9.0/en/setup/dev_env.html) must be installed 


## Usage ## 
### With PX4 SITL and Gazebo ### 

1. Update your takeoff location via environment variables as explained [here](http://dev.px4.io/v1.9.0/en/simulation/gazebo.html#set-custom-takeoff-location). For the RC airfield in the video, the coordinates are:

```sh
export PX4_HOME_LAT=40.091754
export PX4_HOME_LON=-3.695714
```

2. Launch PX4 and MavROS:
```sh
roslaunch px4 mavros_posix_sitl.launch 

```

3. Launch mavros_mission_px4:
```sh
roslaunch mavros_mission_px4 mavros_mission_px4.launch 

```



