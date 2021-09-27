#!/usr/bin/env python3

import rospy
import sys
import math
import numpy as np
from time import time
from sensor_msgs.msg import Imu, MagneticField, NavSatFix

try:
    from ros_openimu.src.aceinna.tools import OpenIMU
except:  # pylint: disable=bare-except
    temp = (sys.path[0])
    temp2 = temp[0:(len(temp)-7)]
    sys.path.append(temp2 + 'src')
    #sys.path.append('./src')
    from aceinna.tools import OpenIMU


class OpenIMUros:
    def __init__(self):
        self.openimudev = OpenIMU()
        self.openimudev.startup()

    def close(self):
        self.openimudev.close()

    '''
    def readimu(self, packet_type):
        readback = self.openimudev.getdata(packet_type)
        return readback
    '''

    def readimu(self):
        readback = self.openimudev.getdata('e2')
        return readback

    def euler_to_quaternion(self,roll, pitch, yaw):
    
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

if __name__ == "__main__":
    rospy.init_node("openimu_driver")

    pub_imu = rospy.Publisher('imu_acc_ar', Imu, queue_size=1)
    pub_mag = rospy.Publisher('imu_mag', MagneticField, queue_size=1)
    pub_gps = rospy.Publisher('gps',NavSatFix,queue_size=1)

    imu_msg = Imu()             # IMU data
    mag_msg = MagneticField()   # Magnetometer data
    gps_msg = NavSatFix()  # Navigation data
    rate = rospy.Rate(10)   # 10Hz
    seq = 0
    frame_id = 'OpenIMU'
    rtk_frame_id = 'RTK_Emlid'
    convert_rads = math.pi /180
    convert_tesla = 10000

    openimu_wrp = OpenIMUros()
    rospy.loginfo("OpenIMU driver initialized.")

    # while not rospy.is_shutdown():
    #     #read the data - call the get imu measurement data
    #     readback = openimu_wrp.readimu()
    #     #publish the data m/s^2 and convert deg/s to rad/s
    #     imu_msg.header.stamp = rospy.Time.now()
    #     imu_msg.header.frame_id = frame_id
    #     imu_msg.header.seq = seq
    #     imu_msg.orientation_covariance[0] = -1
    #     imu_msg.linear_acceleration.x = readback[1]
    #     imu_msg.linear_acceleration.y = readback[2]
    #     imu_msg.linear_acceleration.z = readback[3]
    #     imu_msg.linear_acceleration_covariance[0] = -1
    #     imu_msg.angular_velocity.x = readback[4] / convert_rads
    #     imu_msg.angular_velocity.y = readback[5] / convert_rads
    #     imu_msg.angular_velocity.z = readback[6] / convert_rads
    #     imu_msg.angular_velocity_covariance[0] = -1
    #     pub_imu.publish(imu_msg)

    #     # Publish magnetometer data - convert Gauss to Tesla
    #     mag_msg.header.stamp = imu_msg.header.stamp
    #     mag_msg.header.frame_id = frame_id
    #     mag_msg.header.seq = seq
    #     mag_msg.magnetic_field.x = readback[7] / convert_tesla
    #     mag_msg.magnetic_field.y = readback[8] / convert_tesla
    #     mag_msg.magnetic_field.z = readback[9] / convert_tesla
    #     mag_msg.magnetic_field_covariance = [0,0,0,0,0,0,0,0,0]
    #     pub_mag.publish(mag_msg)

    #     seq = seq + 1
    #     rate.sleep()
    # openimu_wrp.close()         # exit
    while not rospy.is_shutdown():
            #read the data - call the get imu measurement data
        readback = openimu_wrp.readimu()
        #publish the data m/s^2 and convert deg/s to rad/s
        imu_msg.header.stamp.secs= readback[0]
        #imu_msg.header.stamp = rospy.Time.now()
        #imu_msg.header.stamp.nsecs = readback[1]
        imu_msg.header.frame_id = frame_id
        imu_msg.header.seq = seq
        q = openimu_wrp.euler_to_quaternion(readback[2],readback[3],readback[4])
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        imu_msg.orientation_covariance[0] = readback[2]
        imu_msg.orientation_covariance[1] = readback[3]
        imu_msg.orientation_covariance[2] = readback[4]
        #imu_msg.orientation_covariance[0] = -1
        imu_msg.linear_acceleration.x = readback[5]
        imu_msg.linear_acceleration.y = readback[6]
        imu_msg.linear_acceleration.z = readback[7]
        imu_msg.linear_acceleration_covariance[0] = readback[17]
        imu_msg.linear_acceleration_covariance[1] = readback[18]
        imu_msg.linear_acceleration_covariance[2] = readback[19]
        imu_msg.angular_velocity.x = readback[8] *convert_rads
        imu_msg.angular_velocity.y = readback[9] *convert_rads
        imu_msg.angular_velocity.z = readback[10] *convert_rads
        imu_msg.angular_velocity_covariance[0] = readback[20]
        imu_msg.angular_velocity_covariance[1] = readback[21]
        imu_msg.angular_velocity_covariance[2] = readback[22]
        pub_imu.publish(imu_msg)

        # Publish magnetometer data 
        mag_msg.header.stamp = imu_msg.header.stamp
        mag_msg.header.frame_id = frame_id
        mag_msg.header.seq = seq
        mag_msg.magnetic_field.x = readback[11] 
        mag_msg.magnetic_field.y = readback[12] 
        mag_msg.magnetic_field.z = readback[13] 
        mag_msg.magnetic_field_covariance = [0,0,0,0,0,0,0,0,0]
        pub_mag.publish(mag_msg)

        #Publish GNSS data 
        gps_msg.header.stamp = imu_msg.header.stamp
        gps_msg.header.frame_id = rtk_frame_id
        gps_msg.latitude = readback[14]
        gps_msg.longitude = readback[15]
        gps_msg.altitude = readback[16]
        pub_gps.publish(gps_msg)

        seq = seq + 1
        rate.sleep()
    openimu_wrp.close()         # exit




