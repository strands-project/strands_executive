#!/usr/bin/env python
PKG = 'gcal_routine'
NAME = 'gcal_tester'

import rospy
import unittest
import rostest
import sys


from gcal_routine.tools import GCal
from roslib.packages import find_resource


class GCalTest(unittest.TestCase):
    def __init__(self, *args):
        super(GCalTest, self).__init__(*args)
        filename = find_resource(PKG, 'test.json')[0]
        rospy.init_node(NAME)
        self.gcal = GCal(None,
                         None,
                         add_cb=self._added, remove_cb=self._removed,
                         file_name=filename)

    def _added(self, a):
        print '===   added task: %s' % str(a)
        self.assertEqual(a.start_node_id, 'place5')

    def _removed(self, a):
        print '=== removed task: %s' % str(a)
        # in this test we should never call this!
        self.fail()

    def test_update(self):
        self.assertEqual(len(self.gcal.get_task_list()), 0)
        self.assertTrue(self.gcal.update())
        self.assertEqual(len(self.gcal.get_task_list()), 4)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, GCalTest, sys.argv)
