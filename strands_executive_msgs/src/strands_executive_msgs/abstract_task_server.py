#!/usr/bin/env python

import rospy
import actionlib
from strands_executive_msgs.srv import IsTaskInterruptible, CreateTask
from actionlib.msg import TestAction

class AbstractTaskServer(object):
    def __init__(self, name, action_type=TestAction, interruptible=True):         
        self.name = name
        self.action_type = action_type
        self.interruptible = interruptible
        self.server = actionlib.SimpleActionServer(self.name, self.action_type, self.execute, False) 
        self.server.start()
        rospy.Service('%s_is_interruptible' % self.name, IsTaskInterruptible, self.is_interruptible)
        rospy.Service('%s_create' % self.name, CreateTask, self.create)

    def is_interruptible(self, req):
        return self.interuptible

    def create(self, req):
        task = req.task
        task.max_duration = (task.end_before - task.start_after) / 2
        task.action = self.name
        if task.end_node_id is None:
            task.end_node_id = task.start_node_id
        return task

    def execute(self, goal):
        rospy.loginfo("dummy task goal is: %s" % goal) 
        if self.server.is_preempt_requested():
            self.server.set_preempted()
        else:
            self.server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node("abstract_task_server")

# wait for simulated time to kick in as rospy.get_rostime() is 0 until first clock message received
    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    waiter = AbstractTaskServer('abstract')

    rospy.spin()
