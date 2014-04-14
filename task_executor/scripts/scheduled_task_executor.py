#!/usr/bin/env python

import rospy
from Queue import Queue, Empty
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import GetSchedule
from task_executor.base_executor import AbstractTaskExecutor
from threading import Thread
from task_executor.execution_schedule import ExecutionSchedule

class ScheduledTaskExecutor(AbstractTaskExecutor):

    def __init__(self):
        # init node first, must be done before call to super init for service advertising to work
        rospy.init_node("task_executor", log_level=rospy.DEBUG)

        # init superclasses
        super( ScheduledTaskExecutor, self ).__init__()

        # service for scheduler
        schedule_srv_name = 'get_schedule'
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


    def add_task(self, task):
        """ Called with a new task for the executor """
        self.fill_times(task)
        print task
        self.unscheduled_tasks.put(task)

    def call_scheduler(self, tasks):
        resp = self.schedule_srv(tasks)

        # add start times to a dictionary for fast lookup
        task_times = {}
        for (task_id, start_time) in zip(resp.task_order, resp.execution_times):
            print 'task %s will start at %s.%s' % (task_id, start_time.secs, start_time.nsecs)            
            task_times[task_id] = start_time

        # set start times inside of tasks
        for task in tasks:
            task.execution_time = task_times[task.task_id]
            assert task.execution_time >= task.start_after
            assert task.execution_time + task.expected_duration <= task.end_before

        return 

    def schedule_tasks(self):
        loopSecs = 5
        
        while not rospy.is_shutdown():           
            print "scheduling thread %s" % rospy.is_shutdown()      
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
                
                rospy.logdebug('Got a further %s tasks to schedule' % len(unscheduled))

                self.execution_schedule.add_new_tasks(unscheduled)

                self.call_scheduler(self.execution_schedule.get_schedulable_tasks())


            except Empty, e:
                rospy.logdebug('No new tasks to schedule')



    def execute_tasks(self):
        r = rospy.Rate(1) # 1hz
        
        while not rospy.is_shutdown():           

            print "executing thread %s" % rospy.is_shutdown()
            # dequeue tasks to schedule

            # if current schedule empty, create schedule and execute
      
            # if current schedule not empty but no execution, create schedule and execute 

            # if executing, but no new tasks are now tasks
                # - first try sceduling remaing tasks after the executing ne
                # - if unnsuccessful, try rescheduling all

            r.sleep()
        

    # def wait_for_exit(self):
    #     self.scheduling_thread.join()
    #     self.execution_thread.join()

if __name__ == '__main__':
    executor = ScheduledTaskExecutor()        
    rospy.spin()


# create a schedule class which handles blocking until execution and manages the various changes

