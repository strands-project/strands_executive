#!/usr/bin/env python

import rospy
import actionlib
from strands_executive_msgs.msg import Task
from task_executor.msg import *


class TestTaskAction(object):
    def __init__(self):
        rospy.init_node("test_task_action")
        self.server = actionlib.SimpleActionServer('test_task', TestExecutionAction, self.execute, False)
        self.server.start() 

    def execute(self, goal):
        print 'called with goal %s'%goal
        # Do lots of awesome groundbreaking robot stuff here
        self.server.set_succeeded()


if __name__ == '__main__':
    executor = TestTaskAction()    
    rospy.spin()
