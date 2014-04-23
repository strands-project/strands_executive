#!/usr/bin/env python

import rospy
from Queue import Queue, Empty
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import GetSchedule
from task_executor.base_executor import AbstractTaskExecutor
from threading import Thread
from task_executor.execution_schedule import ExecutionSchedule
from operator import attrgetter

class ScheduledTaskExecutor(AbstractTaskExecutor):

    def __init__(self):
        # init node first, must be done before call to super init for service advertising to work
        rospy.init_node("task_executor", log_level=rospy.INFO)

        # init superclasses
        super( ScheduledTaskExecutor, self ).__init__()


        # service for scheduler
        schedule_srv_name = 'get_schedule'
        rospy.logdebug('Waiting for %s service' % schedule_srv_name)
        rospy.wait_for_service(schedule_srv_name)
        self.schedule_srv = rospy.ServiceProxy(schedule_srv_name, GetSchedule)

        # defaults for setting the ends of tasks
        self.default_duration = rospy.Duration.from_sec(60 * 60 * 4)
        
        # storage for tasks which have been added but not considered 
        self.unscheduled_tasks = Queue()
       
        # data structure that manages tasks
        self.execution_schedule = ExecutionSchedule()

        self.scheduling_thread = Thread(target=self.schedule_tasks)    
        self.execution_thread = Thread(target=self.execute_tasks)

        self.running = False

        self.advertise_services()

    def start_execution(self):
        """ Called when overall execution should  (re)start """
        if not self.running:
            self.scheduling_thread.start()    
            self.execution_thread.start()
            self.running = True


    def get_default_end_time(self, start_time):
        return start_time + self.default_duration


    def fill_times(self, task):
        if task.start_after.is_zero():            
            task.start_after = rospy.get_rostime()

        if task.end_before.is_zero():
            task.end_before = self.get_default_end_time(task.start_after)


    def add_tasks(self, tasks):
        """ Called with new tasks for the executor """
        
        for task in tasks:
            self.fill_times(task)
            # print task

        for task in tasks:
            self.unscheduled_tasks.put(task)

    def task_complete(self, task):
        """ Called when the given task has completed execution """
        # pass signal to schedule
        self.execution_schedule.task_complete(task)


    def call_scheduler(self, tasks):
        """ 
        
        Calls scheduler. Reorders the list of tasks in execution order with their execution times set. 
        
        """
        # scheduler seems to need time to start at zero
        min_window = rospy.Time(rospy.get_rostime().secs * 2)
        for task in tasks:
            if task.start_after < min_window:
                min_window = task.start_after

        # turn this time into a duration (since epoch)
        min_window = rospy.Duration(min_window.secs, min_window.nsecs)
        

        # subtrack min window from all win values
        for task in tasks:
            task.start_after = task.start_after - min_window
            task.end_before = task.end_before - min_window


        resp = self.schedule_srv(tasks)

        # add start times to a dictionary for fast lookup
        task_times = {}
        for (task_id, start_time) in zip(resp.task_order, resp.execution_times):
            task_times[task_id] = start_time

        # set start times inside of tasks
        for task in tasks:
            # add min_window back on to starting times
            task.execution_time = task_times[task.task_id] + min_window
            task.start_after = task.start_after + min_window
            task.end_before = task.end_before + min_window

            assert task.execution_time >= task.start_after
            assert task.execution_time + task.expected_duration <= task.end_before

            print 'task %s will start at %s.%s' % (task.task_id, task.execution_time.secs, task.execution_time.nsecs)                        

        tasks.sort(key=attrgetter('execution_time'))
        

    def schedule_tasks(self):
        loopSecs = 5
        
        while not rospy.is_shutdown():           
            # print "scheduling thread %s" % rospy.is_shutdown()      
            try:
                unscheduled = []
                # block until at least one task is available
                unscheduled.append(self.unscheduled_tasks.get(True, loopSecs))
                # now check for any remaining tasks in the queue
                try:
                    while True:
                        unscheduled.append(self.unscheduled_tasks.get(False))
                except Empty, e:
                    pass
                
                rospy.loginfo('Got a further %s tasks to schedule' % len(unscheduled))

                self.execution_schedule.add_new_tasks(unscheduled)

                tasks = self.execution_schedule.get_schedulable_tasks()

                # reorder tasks and add execution information
                self.call_scheduler(tasks)

                # put scheduled tasks back into execution. this will trigger a change in execution if necessary
                self.execution_schedule.set_schedule(tasks)

            except Empty, e:
                rospy.logdebug('No new tasks to schedule')



    def execute_tasks(self):
        wait_time = 1 # 1second
        
        while not rospy.is_shutdown():           

            # print "executing thread %s" % rospy.is_shutdown()
            if(self.execution_schedule.wait_for_execution_change(wait_time)):
                next_task = self.execution_schedule.get_current_task()
                rospy.loginfo('Next task to execute: %s' % next_task)
                if next_task:
                    self.execute_task(next_task)                
                else:
                    rospy.loginfo('Next task was None')

    # def wait_for_exit(self):
    #     self.scheduling_thread.join()
    #     self.execution_thread.join()

if __name__ == '__main__':
    executor = ScheduledTaskExecutor()        
    rospy.spin()


# create a schedule class which handles blocking until execution and manages the various changes

