import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf import TransformListener
import tf
# python file to trannsform the orientation data in imu frame in Winnterwheat dataset to base_link frame
imu_pub = rospy.Publisher("/vectornav/imu/data/correct",Imu,queue_size=10)

def imucbk(data):
    """ callback from IMU topic subscriber and publishes IMU orientation transformed into "base_link" frame

    Args:
        data:data from IMU topic in "sensor_msgs/Imu" message type 
    """
    global tf_listener
    tf_listener = TransformListener()
    p1 = PoseStamped()
    print(data.header)
    p1.header = data.header
    p1.pose.orientation = data.orientation
    tf_listener.getLatestCommonTime("/base_link","/imu_link")
    imu_base_link = tf_listener.transformPose("/base_link",p1)
    print(imu_base_link)
    imu_data = Imu()
    imu_data.header = imu_base_link.header
    imu_data.orientation.x = imu_base_link.pose.orientation.x 
    imu_data.orientation.y = imu_base_link.pose.orientation.y
    imu_data.orientation.z = imu_base_link.pose.orientation.z
    imu_data.orientation.w = imu_base_link.pose.orientation.w 
    imu_pub.publish(imu_data)

def listener():
    """Function runs the ROS init node
    """
    rospy.init_node("imu_transform")
    rospy.Subscriber("/vectornav/imu/data",Imu,imucbk)
    rospy.spin()


listener()
