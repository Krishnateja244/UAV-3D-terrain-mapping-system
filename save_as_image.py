#!/usr/bin/env python

import rospy
import cv2
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from os.path import join
# Python file to save the images data in JPG format from ROS image topic

base_path = "/home/krishna/catkin_ws/src/image_files/"

class image_converter:

    def __init__(self, args):
        self.index = 0
        if len(sys.argv) > 1:
            self.index = int(sys.argv[1])
        rospy.init_node('save_img')
        self.bridge = CvBridge()

    def save_image(self):
        """ Function saves images from ROS topic to jpg files 
        """
        while not rospy.is_shutdown():
            #raw_input image files
            data = rospy.wait_for_message('/camera/image_raw', Image)
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            except CvBridgeError as e:
                print(e)
            print ("Saved to: ", base_path+str(self.index)+".jpg")
            cv2.imwrite(join(base_path, "frame{:06d}.jpg".format(self.index)), cv_image)#*255)
            self.index += 1
        rospy.spin()



if __name__=='__main__':
    converter = image_converter(sys.argv)
    converter.save_image()
