#!/usr/bin/env python

import rospy
import actionlib
from strands_executive_msgs.srv import IsTaskInterruptible, CreateTask
from strands_executive_msgs.msg import Task
from strands_executive_msgs.abstract_task_server import AbstractTaskServer
from actionlib.msg import TestAction
from mongodb_store_msgs.msg import StringPair
from yaml import load
from abc import abstractmethod, ABCMeta



class TestActionServer(AbstractTaskServer):

    def __init__(self):
        super(TestActionServer, self).__init__('test_action')

    def execute(self, goal):
        return


if __name__ == '__main__':
    rospy.init_node("test_action_server")

    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    waiter = TestActionServer()

    rospy.spin()
