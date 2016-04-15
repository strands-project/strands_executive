#!/usr/bin/env python

import rospy
from task_executor.testing import TestEntry


def check_task_descriptions(task_descriptions):
    print 'remaining task descriptions: ', task_descriptions
    
def check_time_diffs(time_diffs):
    print 'time diffs: ', time_diffs

if __name__ == '__main__':
    executor = TestEntry('use_all_test')        
    # testing time critical task
    # executor.run_test(check_task_descriptions, test_tasks = 5, time_critical_tasks = 3, time_diffs_fn = check_time_diffs)

    # testing pause / restart
    executor.run_test(check_task_descriptions,  test_tasks = 10, demanded_tasks = 3, time_critical_tasks = 1)    
    
    rospy.spin()