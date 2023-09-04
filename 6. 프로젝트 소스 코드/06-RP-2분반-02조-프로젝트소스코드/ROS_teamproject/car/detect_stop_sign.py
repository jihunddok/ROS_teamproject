#! /usr/bin/env python
# -*- coding:utf-8

import rospy
import cv2, cv_bridge
import numpy as np
import time
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool
from robot_drive_controller import RobotDriveController


MIN_MATCH_COUNT = 15
show_matched_points = True


class ImageMatch:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()


        # Class for extracting Speeded Up Robust Features from an image
        # 이미지로부터 빠른 속도의 특징을 추출하는 클래스
        self.surf = cv2.xfeatures2d.SURF_create(1000)
        # imread 함수는 지정된 파일에서 이미지를로드하여 반환합니다. 누락 된 파일, 부적절한 권한, 지원되지 않거나 잘못된 형식으로 인해 이미지를 읽을 수없는 경우 함수는 빈 행렬 ( Mat :: data == NULL)을 반환합니다 .
        # 이 기능은 파일 확장자가 아닌 내용에 따라 이미지 유형을 결정합니다
        self.stop_sign_img = cv2.imread('stop_sign.png', cv2.IMREAD_COLOR)

        self.match_pub = rospy.Publisher("matches/is_block", Bool)
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.imagecallback)

        self.match = False
        self.count = 0
        self.drive_controller = RobotDriveController()

    def imagecallback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        imageGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # optimize image
        cv2.useOptimized()
        cv2.setUseOptimized(True)

        # keypoints, descriptors	=	cv.Feature2D.detectAndCompute(	image, mask[, descriptors[, useProvidedKeypoints]]	)
        # Detects keypoints and computes the descriptors
        kp1, des1 = self.surf.detectAndCompute(self.stop_sign_img, None)
        kp2, des2 = self.surf.detectAndCompute(imageGray, None)

        index_params = dict(algorithm=1, trees=5)
        search_params = dict(checks=50)

        matches = None
        # FLANN 은 Fast Library for Approximate Nearest Neighbors 약자로,
        # 큰 이미지에서 특징들을 추출하여 구별하기 위한 최적화된 라이브러리 모음
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)


        match_points = []

        for m, n in matches:
            if m.distance < 0.5 * n.distance:
                match_points.append(m)

        outer_dst_pts = np.float32([])

        if len(match_points) > MIN_MATCH_COUNT:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in match_points]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in match_points]).reshape(-1, 1, 2)

            outer_dst_pts = dst_pts

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

            matchesMask = mask.ravel().tolist()

            h, w, d = self.stop_sign_img.shape

            pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)

            dst = None

            try:
                dst = cv2.perspectiveTransform(pts, M)
            except Exception as ex:

                return


            self.match = True

            rospy.loginfo('Stop-Sign Sign detected : %s' % self.match)


        else:
            self.match = False

            matchesMask = None

        draw_params = dict(matchColor=(150,15 ,50), singlePointColor=None, matchesMask=matchesMask, flags=2)

        matches_img = cv2.drawMatches(self.stop_sign_img, kp1, image, kp2, match_points, None, **draw_params)

        if self.match == True and self.count  < 1:
            self.drive_controller.set_velocity(0)
            print('stop!')
            self.count = self.count + 1
            rospy.sleep(3)
        elif self.count >=1:
            self.drive_controller.set_velocity(0.7)
            self.drive_controller.set_angular(0.3)

        else:
            self.drive_controller.set_velocity(1)
            rospy.loginfo('Stop-Sign Sign detected : %s' % self.match)

        self.drive_controller.drive()

        if show_matched_points:
            for pt in outer_dst_pts:
                x, y = pt[0]

                cv2.circle(image, (x, y), 3, (0, 0, 255), -1)

        self.match_pub.publish(self.match)



        cv2.imshow("stop_sign",matches_img)
        cv2.waitKey(3)




if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    im = ImageMatch()
    rospy.spin()
