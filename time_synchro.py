import message_filters
import rospy
from message_filters import TimeSynchronizer, Subscriber
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from novatel_msgs.msg import INSPVAX
# python file solves the timesynchronization problem in Hongkong dataset
imu_sync = rospy.Publisher("/imu/data/sync",Imu,queue_size=10)

def imu_callback(data):
    """ Function receives unsynchronized imudata and publishes the imu data synchronized to gps data 

    Args:
        data : imu data from topic in "Sensor_msgs/Imu" messaage type
    """
    time = data.header.stamp.secs-2
    imu_data = Imu()
    imu_data.header.stamp.secs = time
    imu_data.header.stamp.nsecs = data.header.stamp.nsecs
    imu_data.header.seq = data.header.seq
    imu_data.header.frame_id = data.header.frame_id
    imu_data.orientation = data.orientation
    imu_data.orientation_covariance = data.orientation_covariance
    imu_data.angular_velocity = data.angular_velocity
    imu_data.angular_velocity_covariance = data.angular_velocity_covariance
    imu_data.linear_acceleration = data.linear_acceleration
    imu_data.linear_acceleration_covariance = data.linear_acceleration_covariance
    imu_sync.publish(imu_data)


def listener():
    """ Function runs the ROS init node
    """
    rospy.init_node("synchro")
    rospy.Subscriber("/imu/data",Imu,imu_callback)
    # rospy.Subscriber("novatel_data/inspvax",INSPVAX,nov)
    rospy.spin()

listener()
