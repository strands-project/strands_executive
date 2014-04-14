#!/usr/bin/env python
PKG = 'task_executor'
NAME = 'scheduler_tester'

import rospy
import unittest
import rostest
import sys

from random import random
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import GetSchedule


class TestEntry(unittest.TestCase):

    def test_scheduler(self):           
        task_count = 5
        one_hour_secs = 60 * 60 * 60
        max_duration = rospy.Duration(one_hour_secs)
        start_of_window = rospy.get_rostime()
        end_of_window = start_of_window + rospy.Duration(one_hour_secs * (task_count + 1))

        tasks = []
        for task_id in range(task_count):    
            # create the task from the description
            task = Task()
            task.task_id=task_id
            task.start_node_id=str(task_id)
            task.end_node_id=str(task_id)
            task.start_after = start_of_window
            task.end_before = end_of_window
            task.expected_duration = rospy.Duration(max_duration.secs * random())
            tasks.append(task)

        # get  services
        schedule_srv_name = 'get_schedule'
        rospy.loginfo("Waiting for scheduler service...")
        rospy.wait_for_service(schedule_srv_name)
        rospy.loginfo("Done")        
        schedule_srv = rospy.ServiceProxy(schedule_srv_name, GetSchedule)
        
        try:
            
                    
            # Schedule the tasks
            resp = schedule_srv(tasks)

            # add start times to a dictionary for fast lookup
            task_times = {}
            for (task_id, start_time) in zip(resp.task_order, resp.execution_times):
                task_times[task_id] = start_time

            # set start times inside of tasks
            for task in tasks:
                task.execution_time = task_times[task.task_id]
                assert task.execution_time >= task.start_after
                assert task.execution_time + task.expected_duration <= task.end_before



        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        

if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestEntry, sys.argv)

