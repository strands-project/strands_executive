#!/usr/bin/env python
PKG = 'gcal_routine'
NAME = 'gcal_tester'

import rospy
import unittest
import rostest
import sys

from strands_executive_msgs.srv import AddTasks
from gcal_routine.tools import GCal
from gcal_routine.tools import rostime_str
from gcal_routine.queue_routine_runner import GCalRoutineRunner
from roslib.packages import find_resource


class GCalTest(unittest.TestCase):
    def __init__(self, *args):
        super(GCalTest, self).__init__(*args)
        self.filename = find_resource(PKG, 'test.json')[0]
        rospy.init_node(NAME)

    def _added(self, a, t):
        print '===   added task: %s' % str(t)
        self.assertEqual(t.start_node_id, 'place5')

    def _removed(self, a, t):
        print '=== removed task: %s' % str(t)
        # in this test we should never call this!
        self.fail()

    def test_gcal(self):
        gcal = GCal(None,
                    None,
                    add_cb=self._added, remove_cb=self._removed,
                    file_name=self.filename)
        self.assertEqual(len(gcal.get_task_list()), 0)
        added = []
        removed = []
        self.assertTrue(gcal.update(added, removed))
        self.assertEqual(len(added), 4)
        self.assertEqual(len(gcal.get_task_list()), 4)
        gcal.shift_to_now()

    def test_runner(self):
        add_tasks_srv_name = '/task_executor/add_tasks'
        rospy.loginfo("Waiting for task_executor service...")
        rospy.wait_for_service(add_tasks_srv_name)
        rospy.loginfo("Done")
        add_tasks_srv = rospy.ServiceProxy(add_tasks_srv_name, AddTasks)

        runner = GCalRoutineRunner(add_tasks_srv, update_wait=0.5)
        gcal = GCal(None,
                    None,
                    add_cb=runner.add_task, remove_cb=runner.remove_task,
                    file_name=self.filename)
        added = []
        removed = []
        self.assertTrue(gcal.update(added, removed))
        gcal.shift_to_now()
        self.assertEqual(len(gcal.get_task_list()), 4)
        rospy.sleep(15)
        # after this time, the first task should already
        # be sent to the scheduler
        self.assertEqual(len(runner.waiting_tasks), 3)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, GCalTest, sys.argv)
