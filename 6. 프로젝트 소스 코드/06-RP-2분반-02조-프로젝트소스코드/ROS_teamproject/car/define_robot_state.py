#!/usr/bin/env python

import rospy
from smach import State
from time import sleep
from detect_blocking_bar import BlockDetector
from lane_follower import LineTracer
from robot_drive_controller import RobotDriveController
import time
import math
from detect_stop_sign import ImageMatch


class SettingLane(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        # line_finder = BlockDetector()
        return 'success'


class DetectBlockingBar(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        block_finder = BlockDetector()
        countf = 0
        while True :
            print(len(block_finder.contours))

            if len(block_finder.contours) == 0 and countf > 3:
                block_finder.drive_controller.set_velocity(1)

                print('go')

                start_time = time.time() +3

                while True:
                    block_finder.drive_controller.drive()
                    if time.time() - start_time > 0:
                        break

                block_finder.drive_controller.set_velocity(0)

                return 'success'

            elif len(block_finder.contours) != 0 and countf > 3:
                block_finder.drive_controller.set_velocity(0)
                print('stop')

            else :
                print('detecting...')
            countf = countf +1
            rospy.sleep(0.3)


class LaneTrace(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        left_line = LineTracer('my_left_camera/rgb/image_raw')
        right_line = LineTracer('my_right_camera/rgb/image_raw')
        stop_line = LineTracer('camera/rgb/image_raw')
        drive_controller = RobotDriveController()
        rate = rospy.Rate(10)
        count = 0

        while not rospy.is_shutdown():
            cx = (left_line.cx + right_line.cx) / 2
            err = -float(cx) / 100

            if stop_line.area > 9000.0: #is stop line?

                drive_controller.set_velocity(0)
                drive_controller.set_angular(0)
                count = count + 1 #stop line count
                print('stop!')
                print(count)
                rospy.sleep(3)

            if count == 4: # in 4 lane

                drive_controller.set_velocity(0)
                drive_controller.set_angular(-0.2)


                start_time = time.time() + 1.5
                # set accurate angle to drive to 4 lane

                while True:
                    drive_controller.drive()
                    if time.time() - start_time > 0:
                        count = count + 1
                        break

                drive_controller.set_velocity(1)
                drive_controller.set_angular(0)
                drive_controller.drive()

                start_time = time.time() + 5

                while True:
                    drive_controller.drive()
                    if time.time() - start_time > 0:
                        count = count + 1
                        break
                print("end")

                drive_controller.set_velocity(0)

            if abs(err) > 0.17 :
                drive_controller.set_velocity(0.4)
                drive_controller.set_angular(err)
                drive_controller.drive()

            elif abs(err) < 0.17 :
                drive_controller.set_velocity(1)
                drive_controller.set_angular(err)
                drive_controller.drive()

            rate.sleep()
        return 'success'


class DetectObstacle(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        detect_obstacle = DetectObstacle()

        # foward is free, right angle is free, then go foward.
        if detect_obstacle.range_ahead > 2 or detect_obstacle.range_right > 2 or \
                ((math.isnan(detect_obstacle.range_ahead)) and math.isnan(detect_obstacle.range_right)):
            value = False
            detect_obstacle.stop_pub.publish(value)
            #    self.drive_controller.drive_forward(1)
            print('go')
        # else stop.
        else:
            value = True
            detect_obstacle.stop_pub.publish(value)
            #    self.drive_controller.set_velocity(0)
            print('stop')

        return 'success'


class DetectStopSign(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        ic = ImageMatch()
        while True:
            if ic.match == True:
                ic.drive_controller.set_velocity(0)
                rospy.sleep(3)
                break
            else :
                continue

        return 'success'


class RightAngleParking(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        return 'success'


class ParallelParking(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        return 'success'


