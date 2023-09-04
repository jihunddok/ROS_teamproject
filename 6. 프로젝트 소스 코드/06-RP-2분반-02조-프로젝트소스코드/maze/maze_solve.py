#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, math
import copy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


# 좌표값 저장을 위한 클래스
class Stack(list):
    push = list.append
    total_count = 0  # 스택 쌓은 횟수

    def is_empty(self):  # 데이터가 없는지 확인
        if not self:
            return True
        else:
            return False

    def peek(self):  # 최상단 데이터 확인
        return self[-1]


class MazeStart:  # 미로 탈출 시작시 메인 클래스, 미로 탈출, 좌표값 저장, 트랙백 등의 기능 실행
    def __init__(self):
        self.s = MazeEscape()  # 미로 탈출
        self.b = ReTrace()  # 트랙백
        self.goal = False  # 미로탈출 성공여부를 알려주는 변수

    def maze_start(self):
        while not rospy.is_shutdown():
            # 맵의 절대좌표가 목표값과 일치하고 미로탈출 성공여부가 참일 경우 (-> 트랙백 종료)
            if (-8.55 < self.s.map_x < -7.55) and (
                    5.05 < self.s.map_y < 5.55) and self.goal is True:
                print('성공!!!!프로그램을 종료 합니다!!')
                break  # 프로그램 종료
            # 맵의 절대좌표가 탈출성공 좌표와 일치하거나 미로탈출 성공여부가 참일경우 (-> 미로탈출 성공 -> 리트레이스(최적경로 탐색) 시작)
            if ((7.75 < self.s.map_x < 8.15) and (
                    -5.35 < self.s.map_y < -4.95)) or self.goal is True:
                self.goal = True  # 탈출성공여부를 참으로 바꾼다.
                self.b.retrace()  # 리트레이스 실행
            else:
                self.s.maze_escape()  # 미로탈출 실행
                self.s.data()  # 좌표값 저장함수 실행


# 미로 탈출 클래스(1차 주행)
class MazeEscape:
    # stack
    xStack = Stack()  # 스택클래스의 스택기능을 이용해서 스택x 선언
    yStack = Stack()
    # abs
    map_x = 0  # 맵의 절대 좌표
    map_y = 0

    def __init__(self):
        # twist & Goal
        self.driving_forward = True  # 직진 여부 판단
        self.rotate = False  # 회전 여부 판단
        self.first_check = True  # 미로 처음 진입시 참
        self.turn = Twist()
        self.twist = Twist()
        self.rate = rospy.Rate(10)

        # scan(인식)
        self.range_ahead = 1  # 정면
        self.range_left = 1  # 왼쪽
        self.range_right = 1  # 오른쪽

        # odom(터틀봇의 현재좌표)
        self.odom_x = 0
        self.odom_y = 0

        # topic
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)  # odom
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

    # function scan
    def scan_callback(self, msg):
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]  # 900 ahead
        self.range_left = msg.ranges[int(len(msg.ranges) / 1.3)]  # 1350 left
        self.range_right = msg.ranges[len(msg.ranges) / 4]  # 450 right

    # function odom
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.map_x = self.odom_x - 7.92
        self.map_y = self.odom_y + 5.30

    # 회전시에 좌표값을 저장하는 기능
    def data(self):
        if self.rotate is True:  # 회전 한다면
            # 절대 좌표값을 스택에 넣어줌
            self.xStack.push(self.map_x)
            self.yStack.push(self.map_y)
            print('회전 좌표값 저장')
            Stack.total_count = Stack.total_count + 1  # total_count 증가

    # 미로 탈출시 사용하는 기능
    def maze_escape(self):
        # 처음 미로에 진입했을때 아래 조건문 실행
        if self.first_check is True:
            # 90도 회전해서 미로에 진입
            for i in range(13):
                first_start = Twist()
                first_start.angular.z = -math.radians(90)
                self.cmd_vel_pub.publish(first_start)
                self.rate.sleep()
            self.first_check = False
        else:
            # rotate = True -> 1초 동안 회전 수행
            if self.rotate is True:
                for i in range(10):
                    self.cmd_vel_pub.publish(self.turn)
                    self.rate.sleep()
                self.rotate = False
            else:
                # 직진 여부 판단 변수가 참일때
                if self.driving_forward is True:
                    if self.range_ahead < 0.7:  # 정면거리가 0.7이하이면 주행을 멈춰라 -> rotate 수행
                        self.driving_forward = False
                else:
                    if self.range_ahead > 0.7:  # 정면거리가 0.7 이상이면 직진
                        self.driving_forward = True
                twist = Twist()
                # driving_forward = True 이면 선속도 1로 직진하고 아닐경우에 조건에 맞게 회전을 수행
                if self.driving_forward is True:
                    twist.linear.x = 1
                    # 회전시 왼쪽과 오른쪽 인식값 중 더 먼지점으로 회전 수행( 거리가 먼 벽쪽으로 회전)
                    if self.range_left > self.range_right:
                        self.turn.angular.z = math.radians(90)
                    elif self.range_left < self.range_right:
                        self.turn.angular.z = -math.radians(90)
                else:
                    self.rotate = True
                self.cmd_vel_pub.publish(twist)
                self.rate.sleep()


# 최적경로 재탈출 수행
class ReTrace:
    def __init__(self):
        self.s = MazeEscape()  # 미로탈출 객체 지정
        # retrace
        self.first_retrace = True  # 백트래킹 시작여부
        self.re_drive = True  # 직진
        self.re_rotate = False  # 회전
        self.rate = rospy.Rate(10)

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.s.odom_callback)  # odom
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.s.scan_callback)

    # 2차 주행 retrace 실행
    def retrace(self):
        if self.first_retrace is True:  # 첫 진입시
            # 180도 회전해서 미로 안쪽으로 바라보게 하기
            for i in range(21):
                first_start = Twist()
                first_start.angular.z = -math.radians(90)
                self.cmd_vel_pub.publish(first_start)
                self.rate.sleep()
            self.first_retrace = False
        else:
            twist = Twist()
            # 스택에 쌓았던 값들의 갯수만큼 리트레이스 수행
            for i in range(0, Stack.total_count, 1):
                self.re_drive = True
                # 현재 터틀봇의 위치 좌표와 스택에 쌍은 값들 중 0.25의  오차만큼 일치한다면
                if (self.s.xStack[i] - 0.25 <= self.s.map_x <= self.s.xStack[i] + 0.25) and (
                        self.s.yStack[i] - 0.25 <= self.s.map_y < self.s.yStack[
                    i] + 0.25):
                    self.re_drive = False  # 정지
                    self.re_rotate = True  # 회전 수행
                    if self.re_rotate is True:
                        if self.re_drive is False:
                            # 마찬가지로 왼쪽과 오른쪽 중 더 먼곳으로 회전한다. -> 최적의 경로로 리트레이스하는것을 확인
                            # 회전후 다시 직진시키기 위해 적정거리를 임의로 이동시킨다.
                            if self.s.range_left > self.s.range_right:
                                for i in range(10):
                                    search = Twist()
                                    search.angular.z = math.radians(90)
                                    self.cmd_vel_pub.publish(search)
                                    self.rate.sleep()
                                for i in range(5):  # 0.5초동안
                                    twist.linear.x = 1  # 선속도 1을 움직여라
                                    self.cmd_vel_pub.publish(twist)  # 실행해라
                                    self.rate.sleep()
                            elif self.s.range_left < self.s.range_right:
                                for i in range(10):
                                    search = Twist()
                                    search.angular.z = -math.radians(90)
                                    self.cmd_vel_pub.publish(search)
                                    self.rate.sleep()
                                for i in range(5):  # 0.5초동안
                                    twist.linear.x = 1  # 선속도 1을 움직여라
                                    self.cmd_vel_pub.publish(twist)  # 실행해라
                                    self.rate.sleep()
                        # 직진 수행
                        self.re_drive = True
                        self.re_rotate = False
                else:  # 아니라면 계속 선속도 1로 달려라
                    if self.re_rotate is False:
                        if self.re_drive is True:
                            print('최적경로 탐색중 ')
                            twist.linear.x = 1
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()


# main
if __name__ == "__main__":
    rospy.init_node('maze_module')
    scan_move = MazeStart()
    scan_move.maze_start()

