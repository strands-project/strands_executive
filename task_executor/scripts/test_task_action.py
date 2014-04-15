#!/usr/bin/env python

import rospy
import actionlib
from strands_executive_msgs.msg import Task
from task_executor.msg import *
from topological_navigation.msg import GotoNodeAction

class TestTaskAction(object):
    def __init__(self):
        rospy.init_node("test_task_action", log_level=rospy.INFO)
        # self.task_server = actionlib.SimpleActionServer('test_task', TestExecutionAction, self.execute, False)
        # self.task_server.start() 
        self.nav_server = actionlib.SimpleActionServer('topological_navigation', GotoNodeAction, execute_cb = self.nav_callback, auto_start = False)
        self.nav_server.start() 


    def execute(self, goal):
        print 'called with goal %s'%goal
        rospy.sleep(1)
        print 'done here'
        self.task_server.set_succeeded()

    def nav_callback(self, goal):
        print 'called with nav goal %s'%goal
        rospy.sleep(1)
        print 'done nav'
        self.nav_server.set_succeeded()


if __name__ == '__main__':
    executor = TestTaskAction()    
    rospy.spin()
