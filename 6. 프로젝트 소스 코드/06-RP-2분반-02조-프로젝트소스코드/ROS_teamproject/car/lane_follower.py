#!/usr/bin/env python

# Lane follow algorithm made from LineTracer
# use two camera.

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from robot_drive_controller import RobotDriveController

import numpy

class LineTracer:
    def __init__(self, image_topic):

        self.bridge = cv_bridge.CvBridge()
        self.image_pub = rospy.Publisher(image_topic + "/circle", Image, queue_size=1)
        # publisher that markup point on traced line
        # (turtle bot use two camera to detect lines. finally, two points will be marked up.
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)

        self.cx = 0

        self.stop_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback2)
        # subscriber that subscribe camera/rgb/image_raw topic
        # to detect stop line

        self.area = 0;
        # Threshold that Distinguish is stop line

    def image_callback(self, msg):
        origin_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # convert camera image from camera (rgb) to using in opencv img.(bgr 8).

        hsv_image = cv2.cvtColor(origin_image, cv2.COLOR_BGR2HSV)
        # convert opencv img(bgr8) to hsv img.

        h, s, v = cv2.split(hsv_image)
        # mapping each value to h, s, v from hsv img.
        # h, s  not used.

        v = cv2.inRange(v, 210, 220)
        # detecting yellow color line

        M = cv2.moments(v)
        # list for markup points that detected object


        if M['m00'] > 0:
            # usage moments tutorials from opencv 3.3.1 documents
            # markup point to center of gravity
            self.cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(origin_image, (self.cx, cy), 20, (0, 255, 0), -1)
            self.cx = self.cx - 320


        origin_image = self.bridge.cv2_to_imgmsg(origin_image)
        # convert cv image to img msg.
        self.image_pub.publish(origin_image)
        # publish image topic

    def image_callback2(self, msg):
        # image callback2 that define operation that detecting stop line
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # set range what is white
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([0, 0, 255])

        # set mask what I defined
        mask = cv2.inRange(hsv, lower_white, upper_white)


        # set mask range
        h, w = mask.shape

        mask[0:h * 3 / 5, 0:w] = 0
        mask[h - (h / 8):h, 0:w] = 0
        mask[0:h, 0:w / 4] = 0
        mask[0:h, w - (w / 4):w] = 0


        # image , threshold
        # set mask image to binary

        # (over 127, to 255, lower , 0) => binary image

        ret, thr = cv2.threshold(mask, 127, 255, 0)
        _, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # markup contours masked image

        if len(contours) <= 0:
            return  # not found

        cnt = contours[0]

        self.area = cv2.contourArea(cnt)



        # detect rectangle image to load image from contoured image
        x, y, w, h = cv2.boundingRect(cnt)
        mask = cv2.rectangle(mask, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.drawContours(mask, [cnt], 0, (255, 255, 0), 1)
        cv2.waitKey(3)

# main func for test module (in state machine, doesn't used)
if __name__ == '__main__':
    rospy.init_node('lanetrace')
    left_line = LineTracer('my_left_camera/rgb/image_raw')
    right_line = LineTracer('my_right_camera/rgb/image_raw')
    stop_line = LineTracer('camera/rgb/image_raw')
    drive_controller = RobotDriveController()
    rate = rospy.Rate(20)
    count = 0
    while not rospy.is_shutdown():
        cx = (left_line.cx + right_line.cx)/2
        err = -float(cx)/100

        if stop_line.area > 9000.0:
            drive_controller.set_velocity(0)
            drive_controller.set_angular(0)
            count = count + 1
            print('stop!')
            print(count)
            rospy.sleep(3)

        if count == 4 :
            drive_controller.set_velocity(1)
            drive_controller.set_angular(0)
            drive_controller.drive()

        if abs(err) > 0.20 and stop_line.area < 9000.0:
            drive_controller.set_velocity(0.4)
            drive_controller.set_angular(err)
            drive_controller.drive()

        elif abs(err) <= 0.20 and stop_line.area < 9000.0:
            drive_controller.set_velocity(1)
            drive_controller.set_angular(err)
            drive_controller.drive()

        rate.sleep()

    rospy.spin()

