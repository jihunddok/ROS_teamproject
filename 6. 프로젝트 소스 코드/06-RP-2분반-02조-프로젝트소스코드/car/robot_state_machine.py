#!/usr/bin/env python

# set state machine that operate sequential execute
import rospy
from smach import StateMachine
from define_robot_state import SettingLane, DetectBlockingBar, LaneTrace,DetectStopSign

if __name__ == "__main__":
    rospy.init_node('test_node')
    driving_test_site = StateMachine(outcomes=['success'])
    with driving_test_site:
        StateMachine.add('SETTING_LANE', SettingLane(), transitions={'success': 'DETECT_BLOCKING_BAR'})
        StateMachine.add('DETECT_BLOCKING_BAR', DetectBlockingBar(), transitions={'success': 'LANE_TRACE'})
        StateMachine.add('LANE_TRACE', LaneTrace(), transitions={'success': 'success'})
        #StateMachine.add('LANE_TRACE', LaneTrace(), transitions={'success': 'DETECT_STOP_SIGN'})
        # StateMachine.add('RIGHT_ANGLE_PARKING', RightAngleParking(), transitions={'success': 'DETECT_OBSTACLE'})
        # StateMachine.add('DETECT_OBSTACLE', DetectObstacle(), transitions={'success': 'PARALLEL_PARKING'})
        # StateMachine.add('PARALLEL_PARKING', ParallelParking(), transitions={'success': 'DETECT_STOP_SIGN'})
        #StateMachine.add('DETECT_STOP_SIGN', DetectStopSign(), transitions={'success': 'LANETRACE'})
        # StateMachine.add('') ....

    driving_test_site.execute()
    rospy.spin()
