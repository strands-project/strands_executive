#!/usr/bin/env python

import rospy
import actionlib
from strands_executive_msgs.srv import IsTaskInterruptible, CreateTask
from strands_executive_msgs.msg import Task
from actionlib.msg import TestAction
from mongodb_store_msgs.msg import StringPair
from yaml import load
from abc import abstractmethod, ABCMeta


class AbstractTaskServer(object):

    __metaclass__ = ABCMeta

    def __init__(self, name, action_type=TestAction, interruptible=True):
        self.name = name
        self.action_type = action_type
        self.interruptible = interruptible
        self.server = actionlib.SimpleActionServer(self.name,
                                                   self.action_type,
                                                   self.execute, False)
        self.server.register_preempt_callback(self.preempt_cb)
        self.server.start()
        rospy.Service('%s_is_interruptible' % self.name,
                      IsTaskInterruptible, self._is_interruptible)
        rospy.Service('%s_create' % self.name, CreateTask, self.create)

    def preempt_cb(self):
        """
        overwrite this callback to receive preemption notification
        """
        return

    def _is_interruptible(self, req):
        return self.interruptible

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
        """
        implement the factory service. Overwrite to provide a
        task specific factory method. Idea is that this function
        return a fully instantiated task object with all parameters
        set so that the task can be submitted to the scheduler without
        requiring the routine (or other component that submits the task)
        does not need to know about the internal parameters of the task
        itself.
        :param req.yaml: the yaml specification of the task. This can be
            underspecified. All slots in the yaml are transferred into
            task object for slots that exist in there.
        """
        task = Task()
        task.action = self.name
        if len(req.yaml) > 0:
            self._fill_slots(load(req.yaml), task)
        return task

    @abstractmethod
    def execute(self, goal):
        """
        abstract method that must be implemented from specialising
        class to realise the task functionality.
        :param goal: the actionlib/Goal object
        """
        return


class TestActionServer(AbstractTaskServer):

    def __init__(self):
        super(TestActionServer, self).__init__('test_action')

    def execute(self, goal):
        return


if __name__ == '__main__':
    rospy.init_node("abstract_task_server")

    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    waiter = TestActionServer()

    rospy.spin()

