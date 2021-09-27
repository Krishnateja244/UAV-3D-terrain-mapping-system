Ethzasl_msf
=====================

Multisensor fusion package used in this project.

To fuse IMU with different type of sensors, run the following:
```
rosbag play xxx.bag
roslaunch msf_updates xxxxx_sensor.launch
rosrun rqt_reconfigure rqt_reconfigure
```



