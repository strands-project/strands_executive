#!/usr/bin/env python

import rospy

import unittest
import rostest
import sys

from task_executor.testing import TestEntry

def check_task_descriptions(task_descriptions):
    print 'remaining task descriptions: ', task_descriptions


class TestWrapper(unittest.TestCase):

    def __init__(self, *args):         
        super(TestWrapper, self).__init__(*args)    

    def list_empty(self, task_descriptions):
        self.assertEquals(task_descriptions, [])

    def check_time_diffs(self, time_diffs):
        pass

    def test_execution(self):
        te = TestEntry('execution_test')        
        test = rospy.get_param('~test', 0)
        if test == 0:
            te.run_test(self.list_empty)
        elif test == 1:
            te.run_test(self.list_empty, test_tasks = 5, pause_count = 3)    
        elif test == 2:
            te.run_test(self.list_empty, test_tasks = 10, time_critical_tasks = 3, time_diffs_fn = self.check_time_diffs)
        

if __name__ == '__main__':
    rostest.rosrun('task_executor', 'executor_tests', TestWrapper, sys.argv)