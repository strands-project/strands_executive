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



    def run_scheduler(self, tasks):    
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
                print 'task %s   window from %s.%s to %s.%s' % (task.task_id, task.start_after.secs, task.start_after.nsecs, task.end_before.secs, task.end_before.nsecs)            
                print 'task %s will start at %s.%s' % (task.task_id, task.execution_time.secs, task.execution_time.nsecs)            

                self.assertTrue(task.execution_time >= task.start_after)
                self.assertTrue(task.execution_time + task.expected_duration <= task.end_before)
                

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def create_tasks(self, task_count, start_of_window):           
        one_hour_secs = 60 * 60 * 60
        max_duration = rospy.Duration(one_hour_secs)
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

        return tasks

    def test_start_at_zero(self):        
        self.run_scheduler(self.create_tasks(5, rospy.Time(0)))


    def test_start_at_now(self):
        """ this fails as time needs to start from zero """
        self.run_scheduler(self.create_tasks(5, rospy.get_rostime()))
        

if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestEntry, sys.argv)

