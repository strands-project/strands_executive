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


    # def test_lots_of_tasks(self):       
    #     te = TestEntry('lots_of_tasks')        
    #     te.run_test(self.list_empty)

    def test_execution_pause(self):
        # testing pause / restart
        te = TestEntry('execution_pause')        
        te.run_test(self.list_empty, test_tasks = 5, pause_count = 3)    
    

if __name__ == '__main__':
    rostest.rosrun('task_executor', 'executor_tests', TestWrapper, sys.argv)