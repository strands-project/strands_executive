#!/usr/bin/env python
from __future__ import division

PKG = 'task_executor'
NAME = 'scheduler_tester'

import rospy
import unittest
import rostest
import sys
from math import ceil

from random import random
from strands_executive_msgs.msg import Task, DurationList, DurationMatrix
from strands_executive_msgs.srv import GetSchedule


class TestEntry(unittest.TestCase):
# class TestEntry():

    def run_scheduler(self, tasks, earliest_start, first_task, durations):    
        # get  services
        schedule_srv_name = 'get_schedule'
        rospy.loginfo("Waiting for scheduler service...")
        rospy.wait_for_service(schedule_srv_name)
        rospy.loginfo("Done")        
        schedule_srv = rospy.ServiceProxy(schedule_srv_name, GetSchedule)
        
        try:
            
                    
            # Schedule the tasks
            resp = schedule_srv(tasks, earliest_start, first_task, durations)

            print resp

            if len(resp.task_order) > 0:

                # add start times to a dictionary for fast lookup
                task_times = {}
                for (task_id, start_time) in zip(resp.task_order, resp.execution_times):
                    task_times[task_id] = start_time



                # set start times inside of tasks
                for task in tasks:
                    task.execution_time = task_times[task.task_id]
                    print 'task %s   window from %s.%s to %s.%s' % (task.task_id, task.start_after.secs, task.start_after.nsecs, task.end_before.secs, task.end_before.nsecs)            
                    print 'task %s will start at %s.%s' % (task.task_id, task.execution_time.secs, task.execution_time.nsecs)            

                    self.assertGreaterEqual(task.execution_time, task.start_after)
                    self.assertLessEqual(task.execution_time + task.max_duration, task.end_before)                
            else:                
                self.fail("Couldn't find a scheduler")

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def create_tasks_in_single_window(self, task_count, start_of_window):           
        one_hour_secs = 60 * 60 * 60
        max_duration = rospy.Duration(one_hour_secs)
        end_of_window = start_of_window + rospy.Duration(one_hour_secs * (task_count * 2))

        print "creating %s tasks of length %s to fit into %s" % (task_count, rospy.Duration(max_duration.secs/2).secs, (end_of_window - start_of_window).secs)


        tasks = []
        for task_id in range(1, task_count+1):    
            # create the task from the description
            task = Task()
            task.task_id=task_id
            task.start_node_id=str(task_id)
            task.end_node_id=str(task_id)
            task.start_after = start_of_window
            task.end_before = end_of_window
            task.max_duration = rospy.Duration(max_duration.secs/2)
            tasks.append(task)

        dm = DurationMatrix()
        for i in range(task_count):
            dm.durations.append(DurationList())
            for j in range(task_count):
                dm.durations[-1].durations.append(rospy.Duration(1.0))

        # ['tasks', 'earliest_start', 'first_task', 'durations']
        return (tasks, start_of_window, 0, dm)


    def create_tasks_in_two_windows(self, task_count, start_of_first_window):           
        one_hour_secs = 60 * 60 * 60
        max_duration = rospy.Duration(one_hour_secs)

        first_window_count = int(ceil(task_count / 2))
        second_window_count = task_count // 2
        self.assertEquals(task_count, first_window_count + second_window_count)

        end_of_first_window = start_of_first_window + rospy.Duration(one_hour_secs * (first_window_count + 1))

        tasks = []
        for task_id in range(first_window_count):    
            # create the task from the description
            task = Task()
            task.task_id=task_id
            task.start_node_id=str(task_id)
            task.end_node_id=str(task_id)
            task.start_after = start_of_first_window
            task.end_before = end_of_first_window
            task.max_duration = rospy.Duration(max_duration.secs * random())
            tasks.append(task)

        window_interval = max_duration
        start_of_second_window = end_of_first_window + window_interval
        end_of_second_window = start_of_second_window + rospy.Duration(one_hour_secs * (second_window_count + 1))        
        
        for task_id in range(first_window_count, first_window_count+ second_window_count):    
            # create the task from the description
            task = Task()
            task.task_id=task_id
            task.start_node_id=str(task_id)
            task.end_node_id=str(task_id)
            task.start_after = start_of_second_window
            task.end_before = end_of_second_window
            task.max_duration = rospy.Duration(max_duration.secs * random())
            tasks.append(task)

        dm = DurationMatrix()
        for i in range(task_count):
            dm.durations.append(DurationList())
            for j in range(task_count):
                dm.durations[-1].durations.append(rospy.Duration(1.0))    

        self.assertEquals(task_count, len(tasks))
        return (tasks, start_of_first_window, 0, dm)


    def test_start_at_zero_one_window(self):        
        self.run_scheduler(*self.create_tasks_in_single_window(10, rospy.Time(0)))

    def test_start_at_zero_two_windows(self):        
        self.run_scheduler(*self.create_tasks_in_two_windows(5, rospy.Time(0)))


    def test_start_at_now(self):
        """ this fails as time needs to start from zero """
        # discard nanos from time as they are not respected by the scheduler
        self.run_scheduler(*self.create_tasks_in_single_window(6, rospy.Time(rospy.get_rostime().secs)))
        

if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestEntry, sys.argv)
    # test = TestEntry()
    # test.test_start_at_now()

    # rospy.spin()


