import rospy
from sensor_msgs.msg import NavSatFix
import numpy as np 

## Python file to add gaussian noise to GPS readings to create a noisy GPS readings

gps_noisy = rospy.Publisher("/noise_gps",NavSatFix,queue_size=1000)

def gpscbk(data):
    """ Function receives GPS data from topic and publishes a new topic with noisy GPS data

    Args:
        data: GPS data from topic with "Sensor_msgs/NavsatFix" message type
    """
    global pub1
    time = data.header.stamp
    lat = data.latitude
    lon = data.longitude
    alt = data.altitude
    # Adding Gaussian noise
    noise = np.random.normal(0,0.0000001)
    print(noise)

    gps_sen = NavSatFix()
    gps_sen.header.stamp = time
    gps_sen.latitude = lat+noise
    gps_sen.longitude = lon+noise
    gps_sen.altitude = alt+noise
    gps_noisy.publish(gps_sen)

def listener():
    """ Function runs thee ROS init node
    """
    rospy.init_node("gen_noise_gps")
    rospy.Subscriber("/vectornav/gps/data",NavSatFix,gpscbk)
    rospy.spin()

listener()
