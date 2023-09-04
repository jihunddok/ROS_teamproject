#! /usr/bin/env python

import rospy
import numpy as np
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from robot_drive_controller import RobotDriveController

class BlockDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.block_pub = rospy.Publisher('detect/block', Bool, queue_size=1)
        self.drive_controller = RobotDriveController()
        self.contours = [] #list that judge object

    def image_callback(self, msg):
        origin_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # convert image to gray image ( red, white)
        lower_red = np.array([0, 0, 90])
        upper_red = np.array([5, 5, 110])
        gray_img = cv2.inRange(origin_image, lower_red, upper_red)

        # set mask range in gray image
        h, w = gray_img.shape
        block_bar_mask = gray_img
        block_bar_mask[0:180, 0:w] = 0
        block_bar_mask[240:h, 0:w] = 0

        # count object into contours image that Binarization red image in mask image.
        block_bar_mask, self.contours, hierarchy = cv2.findContours(block_bar_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

if __name__ == '__main__':
    rospy.init_node('test_node')
    line_finder = BlockDetector()
    rospy.spin()

