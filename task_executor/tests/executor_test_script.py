#!/usr/bin/env python

import rospy

import unittest
import rostest
import sys

from task_executor.testing import TestEntry
from strands_executive_msgs.msg import Task, TaskEvent

def check_task_descriptions(task_descriptions):
    print 'remaining task descriptions: ', task_descriptions


class TestWrapper(unittest.TestCase):

    def __init__(self, *args):         
        super(TestWrapper, self).__init__(*args)    

    def list_empty(self, task_descriptions):
        self.assertEquals(task_descriptions, [])

    def check_time_diffs(self, time_diffs):
        pass

    def check_task_events(self, task_events):
        self.assertEquals(len(task_events), 2)
        # task must start
        self.assertEquals(task_events[0].event, TaskEvent.TASK_STARTED)
        # might be preempted or failed, but not succeeeded
        self.assertIn(task_events[1].event, [TaskEvent.TASK_FAILED, TaskEvent.TASK_PREEMPTED])


    def test_execution(self):
        te = TestEntry('execution_test')        
        test = rospy.get_param('~test', 0)
        if test == 0:
            te.run_test(self.list_empty)
        elif test == 1:
            te.run_test(self.list_empty, test_tasks = 5, pause_count = 3)    
        elif test == 2:
            te.run_test(self.list_empty, test_tasks = 10, time_critical_tasks = 3, time_diffs_fn = self.check_time_diffs)
        elif test == 3:
            te.run_test(self.list_empty, time_critical_tasks = 5, demanded_tasks = 3, test_tasks = 5)            
        elif test == 4:
            te.bad_timings(self.check_task_events)

if __name__ == '__main__':
    rostest.rosrun('task_executor', 'executor_tests', TestWrapper, sys.argv)
