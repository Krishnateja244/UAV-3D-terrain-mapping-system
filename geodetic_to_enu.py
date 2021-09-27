import numpy as np
import pyproj
import scipy.spatial.transform     
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped

# python file to convert Geodetic coordinates to ENU frame coordinates
origin = True
lat_org = 0 
lon_org =0
alt_org =0 

def geodetic2enu(lat, lon, alt, lat_org, lon_org, alt_org):
    """ Function to convert Geodetic coordinates to ENU coordinate frame

    Args:
        lat (float): Latitude in degrees
        lon (float): Longitude in degrees
        alt (float): Altitude in meters
        lat_org (float): Initial latitude in degrees
        lon_org (float): Initial longitude in degrees
        alt_org (float): Initial altitude in meters

    Returns:
        list : X,Y,Z coordinates in ENU frame
    """
    transformer = pyproj.Transformer.from_crs(
        {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
        {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
        )
    x, y, z = transformer.transform( lon,lat,  alt,radians=False)
    x_org, y_org, z_org = transformer.transform( lon_org,lat_org,  alt_org,radians=False)
    vec=np.array([[ x-x_org, y-y_org, z-z_org]]).T

    rot1 =  scipy.spatial.transform.Rotation.from_euler('x', -(90-lat_org), degrees=True).as_matrix()#angle*-1 : left handed *-1
    rot3 =  scipy.spatial.transform.Rotation.from_euler('z', -(90+lon_org), degrees=True).as_matrix()#angle*-1 : left handed *-1

    rotMatrix = rot1.dot(rot3)    
   
    enu = rotMatrix.dot(vec).T.ravel()
    return enu.T

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
        print(lat_org,lon_org,alt_org)
  
    res = geodetic2enu(lat, lon, alt, lat_org, lon_org, alt_org)
    print("converted to ENU : ", res)
    #print("-"*10)
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = time
    msg.pose.pose.position.x = res[0]
    msg.pose.pose.position.y = res[1]
    msg.pose.pose.position.z = res[2]
    gps_to_enu_publish.publish(msg)

def pose_callbk(data):
    """ Callback for Pose subscriber 

    Args:
        data : data from msf_pose in "sensor_msgs/PoseWithCovarianceStamped" message type
    """
    global lat_org,lon_org,alt_org
    print(lat_org,lon_org,alt_org)
    time = data.header.stamp
    lat_x = data.pose.pose.position.x
    lon_y = data.pose.pose.position.y
    alt_z = data.pose.pose.position.z
    print(lat_x,lon_y,alt_z)
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
    gps_to_enu_publish.publish(msg_2)

if __name__ == '__main__':

    rospy.init_node("geodetic_to_ENU")
    rospy.Subscriber("/vectornav/gps/data",NavSatFix,gps_callbk)
    #pub = rospy.Publisher("/vectornav",PoseWithCovarianceStamped,queue_size=1000)
    #rospy.Subscriber("/msf_core/pose",PoseWithCovarianceStamped,pose_callbk)
    gps_to_enu_publish = rospy.Publisher("/gps_enu",PoseWithCovarianceStamped,queue_size=1000)

    rospy.spin()
    
