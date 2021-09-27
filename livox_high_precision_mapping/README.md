## Livox mapping package

In this research, the Livox package is used to generate a 3D map using GNSS/IMU for estimating orientaion an position of points in LIDAR pointclouds.


After data collection from the UAV survey , run livox_mapping.launch to complete the offline mapping.

```
#Play the rosbag file recorded during UAV survey
rosbag play xxxxxxx.bag
#Run the mapping program
roslaunch livox_mapping livox_mapping.launch
```

For more information on this package can be found in : https://github.com/Livox-SDK/livox_high_precision_mapping
