#!/usr/bin/env python

import rospy
from task_executor.utils import TestTaskAction

if __name__ == '__main__':
    rospy.init_node('test_task_action')
    actual_action_duration = rospy.Duration(600)    
    action_server = TestTaskAction(expected_action_duration=actual_action_duration, expected_drive_duration=rospy.Duration(3))
    action_server.start()
    rospy.spin()
