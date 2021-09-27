import numpy as np
import pyproj
import scipy.spatial.transform     
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped

# python file to convert ENU frame coordinatees to Geodetic coordinatees
origin = True
lat_org = 0 
lon_org =0
alt_org =0 


def enu2geodetic(x,y,z, lat_org, lon_org, alt_org):
    """ Function to convert ENU frame coordiantes to geodetic coordinates  

    Args:
        x (float): x-coordiante in meters in ENU
        y (float): Y-coordinate in meters in ENU
        z (float): Z-coordiante in meters in ENU
        lat_org (float): Initial latitude in degrees
        lon_org (float): Initial longitude in degrees
        alt_org (float): Initial altitude in meters

    Returns:
        list : Latitude,Longitude and Altitude
    """
    transformer1 = pyproj.Transformer.from_crs(
        {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
        {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
        )
    transformer2 = pyproj.Transformer.from_crs(
        {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
        {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
        )
    
    x_org, y_org, z_org = transformer1.transform( lon_org,lat_org,  alt_org,radians=False)
    ecef_org=np.array([[x_org,y_org,z_org]]).T
    
    rot1 =  scipy.spatial.transform.Rotation.from_euler('x', -(90-lat_org), degrees=True).as_matrix()#angle*-1 : left handed *-1
    rot3 =  scipy.spatial.transform.Rotation.from_euler('z', -(90+lon_org), degrees=True).as_matrix()#angle*-1 : left handed *-1

    rotMatrix = rot1.dot(rot3)

    ecefDelta = rotMatrix.T.dot( np.array([[x,y,z]]).T )
    ecef = ecefDelta+ecef_org
    lon, lat, alt = transformer2.transform( ecef[0,0],ecef[1,0],ecef[2,0],radians=False)

    return [lat,lon,alt]

def gps_callbk(data):
    """ callback from GPS subscriber 

    Args:
        data : data from GPS sensor in "sensor_msgs/NavsatFix" message type
    """
    global origin
    global lat_org,lon_org,alt_org
    time = data.header.stamp
    lat = data.latitude
    lon = data.longitude
    alt = data.altitude
    if origin :
        lat_org = lat
        lon_org = lon
        alt_org = alt
        origin = False
        print("reference :",lat_org,lon_org,alt_org)


def pose_callbk(data):
    """ Callback for Pose subscriber 

    Args:
        data : data from msf_pose in "sensor_msgs/PoseWithCovarianceStamped" message type
    """
    global lat_org,lon_org,alt_org
    #print(lat_org,lon_org,alt_org)
    time = data.header.stamp
    lat_x = data.pose.pose.position.x
    lon_y = data.pose.pose.position.y
    alt_z = data.pose.pose.position.z
    # print(lat_x,lon_y,alt_z)
    res_1 = enu2geodetic(lat_x,lon_y,alt_z,lat_org,lon_org,alt_org)
    print("corrected_coordinates: ",res_1)
    msg_2 = PoseWithCovarianceStamped()
    msg_2.header.stamp = time
    msg_2.pose.pose.position.x = res_1[0]
    msg_2.pose.pose.position.y = res_1[1]
    msg_2.pose.pose.position.z = res_1[2]
    msg_2.pose.pose.orientation.x = data.pose.pose.orientation.x
    msg_2.pose.pose.orientation.y = data.pose.pose.orientation.y
    msg_2.pose.pose.orientation.z = data.pose.pose.orientation.z
    correct_pose_pub.publish(msg_2)

if __name__ == '__main__':

    rospy.init_node("ENU_to_geodetic")
    rospy.Subscriber("/ublox_node/fix",NavSatFix,gps_callbk)
    #pub = rospy.Publisher("/gpsxyz",PoseWithCovarianceStamped,queue_size=1000)
    rospy.Subscriber("/msf_core/pose",PoseWithCovarianceStamped,pose_callbk)
    correct_pose_pub = rospy.Publisher("/corrected_pose",PoseWithCovarianceStamped,queue_size=10)
    rospy.spin()
    
