#!/usr/bin/env python

import rospy
from Queue import Queue, Empty
from strands_executive_msgs.msg import Task
from task_executor.base_executor import AbstractTaskExecutor


class ScheduledTaskExecutor(AbstractTaskExecutor):

    def __init__(self):
        # init node first, must be done before call to super init for service advertising to work
        rospy.init_node("task_executor")

        # init superclasses
        super( ScheduledTaskExecutor, self ).__init__()

        # defaults for setting the ends of tasks
        self.default_duration = rospy.Duration.from_sec(60 * 60 * 4)
        # self.default_execution_cutoff = 
       
    def get_default_end_time(self, start_time):
        return start_time + self.default_duration


    def fill_times(self, task):
        if task.start_after.is_zero():            
            task.start_after = rospy.get_rostime()

        if task.end_by.is_zero():
            task.end_by = self.get_default_end_time(task.start_after)


    def add_task(self, task):
        """ Called with a new task for the executor """
        self.fill_times(task)
        print task

    def run_executor(self):
        r = rospy.Rate(1) # 1hz
        
        while not rospy.is_shutdown():           

            # dequeue tasks to schedule

            # if current schedule empty, create schedule and execute
      
            # if current schedule not empty but no execution, create schedule and execute 

            # if executing, but no new tasks are now tasks
                # - first try sceduling remaing tasks after the executing ne
                # - if unnsuccessful, try rescheduling all

            r.sleep()
        

if __name__ == '__main__':
    executor = ScheduledTaskExecutor()    
    executor.run_executor()    
