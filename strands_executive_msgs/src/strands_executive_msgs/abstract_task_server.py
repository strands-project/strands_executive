#!/usr/bin/env python

import rospy
import actionlib
from strands_executive_msgs.srv import IsTaskInterruptible, CreateTask
from strands_executive_msgs.msg import Task
from actionlib.msg import TestAction
from mongodb_store_msgs.msg import StringPair
from yaml import load


class AbstractTaskServer(object):
    def __init__(self, name, action_type=TestAction, interruptible=True):
        self.name = name
        self.action_type = action_type
        self.interruptible = interruptible
        self.server = actionlib.SimpleActionServer(self.name,
                                                   self.action_type,
                                                   self.execute, False)
        self.server.start()
        rospy.Service('%s_is_interruptible' % self.name,
                      IsTaskInterruptible, self.is_interruptible)
        rospy.Service('%s_create' % self.name, CreateTask, self.create)

    def is_interruptible(self, req):
        return self.interuptible

    def _fill_slots(self, src, dest):
        for s in dest.__slots__:
            if s in src:
                if s == 'arguments':
                    # special case for attributes
                    for a in src[s]:
                        sp = StringPair()
                        self._fill_slots(a, sp)
                        dest.arguments.append(sp)
                else:
                    # a bit of a hacky way to check if we need to recurse...
                    if str(type(dest.__getattribute__(s))).\
                        startswith('<class ') \
                            or type(dest.__getattribute__(s)) == dict:
                        self._fill_slots(src[s], dest.__getattribute__(s))
                    elif type(dest.__getattribute__(s)) == list:
                        dest.__getattribute__(s).extend(src[s])
                    else:
                        dest.__setattr__(s, src[s])

    def create(self, req):
        task = Task()
        task.max_duration = (task.end_before - task.start_after) / 2
        task.action = self.name
        if task.end_node_id is None:
            task.end_node_id = task.start_node_id
	if len(req.yaml) > 0:
        	self._fill_slots(load(req.yaml), task)
        return task

    def execute(self, goal):
        rospy.loginfo("dummy task goal is: %s" % goal)
        if self.server.is_preempt_requested():
            self.server.set_preempted()
        else:
            self.server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node("abstract_task_server")

    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    waiter = AbstractTaskServer('abstract')

    rospy.spin()
