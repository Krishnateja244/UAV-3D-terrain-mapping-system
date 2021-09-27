UAV 3D terrain mapping system
=======
In this 3D terrain mapping system, data collection from sensors during the UAV survey is done initially and further offline processing of the data is done to build a 3D map. The implementation of this research uses mainly Robotic Operating System(ROS). Therefore, the software libraries used for the implementation are divided into two tasks: Data collection and Offline data processing.

Data collection
-----------

The data collection involves collecting data from Velodyne LIDAR, Aceinna IMU, Emlid Reach M+ and Pixhawk 4. The sensors data is collected using the ROS bag tool. The system needs to be setup with appropriate ROS drivers to receive data from sensors into ROS architecture.

To record the data using the ROS bag tool:
```
rosbag record -a
```

Velodyne LIDAR
----

Download Velodyne ROS driver using:
```
git clone git@github.com:ros-drivers/velodyne.git
```
To run the Velodyne sensor ROS driver:
```
roslaunch velodyne\_pointcloud VLP16_points.launch
```
Aceinna IMU300ZI
---

Get the Aceinna ROS package updated based on the requirement of this research from this repository. This ros\_openimu is forked from 
```
https://github.com/ROS-Aceinna/ros_openimu
```
To run the IMU ROS driver:
```
roslaunch ros_openimu openimu_driver.launch 
```
Emlid M+
---

The NMEA sentences from the Emlid M+ and GNSS UTC can be logged using the nmea_navsat driver. This driver is forked from 
```
 https://github.com/ros-drivers/nmea_navsat_driver
```

To run the driver:
```
roslaunch nmea_navsat_driver nmea_serial_driver.launch
```

MAVROS
---

The Pixhawk flight controller data is logged in ROS architecture to log the camera trigger GPS time and corresponding GNSS coordinates.

Download MAVROS library using :
```
git clone git@github.com:mavlink/mavros.git
```

To launch the MAVROS library:

```
roslaunch mavros px4.launch
```

Offline Data processing
---------------

In this step, the data collected from all the sensors are processed to generate a 3D point cloud. The process involves generating the optimal pose of the UAV by sensor fusion and using it for Georeferencing LIDAR point cloud. Two online datasets are used in this research for performing offline data processing, due to technical difficulties in collecting sensor data from UAV.

Two datasets are:

HongKong Dataset UrbanNav-HK-Data20190428: https://github.com/weisongwen/UrbanNavDataset

Winterwheat Path A: https://vision.eng.au.dk/future-cropping/uav_lidar/

The ROS bags collected in the data collection stage/datasets are played by:
```
rosbag play xxx.bag
``` 
Sensor Fusion
---

The Sensor fusion is implemented using Ethzasal_msf multisensor fusion ROS package based on Extended Kalman Filter (EKF). The sensors fused for current implementation are GNSS and IMU. Therefore, Position_sensor.msf ROS node is utilised in this implementation. 

Get the modified Ethzasl\_msf package with topics configuration based on the Hongkong dataset from this repository. This is a forked repository from https://github.com/ethz-asl/ethzasl_msf.

The instructions to run this package are:

 - Pause the bag file once the position_sensor node receives initial readings.
 - Select the "core_init_filter" in the parameter server to initialize the EKF filter with initial readings
 - Unpause the bag file to continue working of the filter 

To run the package:
```
roslaunch msf_updates position_sensor.launch
rosrun rqt_reconfigure rqt_reconfigure
```
To convert the position data from meters in ENU frame published in "msf_core/pose" topic to geocoordinate representation (Latitude,Longitude, Altitude) :
```
python3 enu_to_geodetic.py 
```
Georefereencing LIDAR point cloud
---

The Livox high precision mapping ROS package is used to map the LIDAR point cloud. The updated package can be downloaded from this repository. This is a forked repository from https://github.com/Livox-SDK/livox_high_precision_mapping

The transformation matrix between imu_link and base_link in the Winterwheat dataset is used to transform UAV data from imu_link to base_link using the ROS TF library.

```
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map base_link
rosrun tf2_ros static_transform_publisher -0.039 -0.008 -0.294 -0.7071 4.32978e-7 0.707106 4.32978e-17 base_link imu_link
python transform.py
```

Then run the mapping application 
```
roslaunch livox_mapping livox_mapping.launch
