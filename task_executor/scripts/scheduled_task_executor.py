#!/usr/bin/env python

import rospy
from Queue import Queue, Empty
from strands_executive_msgs.msg import Task, ExecutionStatus
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

        # topic on which current schedule is broadcast
        self.schedule_publisher = rospy.Publisher('/current_schedule', ExecutionStatus)

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

    def task_demanded(self, demanded_task, currently_active_task):
        """ Called when a task is demanded. self.active_task is the demanded task (and is being executed) and previously_active_task was the task that was being executed (which could be None) """

        rospy.loginfo('Task %s, %s demanded from scheduler' % (demanded_task.task_id, demanded_task.action))

        # save the tasks we had previously
        previously_scheduled = self.execution_schedule.get_schedulable_tasks()

        # clear all tasks that were previously scheduled
        self.execution_schedule.clear_schedule()

        # try to schedule them back in 
        self.execution_schedule.add_new_tasks([demanded_task])
        # put scheduled tasks back into execution. this will trigger a change in execution if necessary
        self.execution_schedule.set_schedule([demanded_task])


        # wait until the demanded task has been taken off for execution, otherwise we run into problems with get_schedulable_tasks
        while self.execution_schedule.get_execution_queue_length() > 0:
            rospy.sleep(0.1)
            # rospy.loginfo('SLEEP')


        # now try to put the other tasks back in
        if self.try_schedule(previously_scheduled):
            rospy.loginfo('Was able to reinstate tasks after demand')
            if self.try_schedule([currently_active_task]):
                rospy.loginfo('Was also able to reinstate previously active task after demand')
        else:
            rospy.loginfo('Was NOT able to reinstate tasks after demand')


    def call_scheduler(self, tasks):
        """ 
        
        Calls scheduler. Reorders the list of tasks in execution order with their execution times set. 
        
        """
        resp = self.schedule_srv(tasks)

        if len(resp.task_order) > 0:

            # add start times to a dictionary for fast lookup
            task_times = {}
            for (task_id, start_time) in zip(resp.task_order, resp.execution_times):
                task_times[task_id] = start_time

            # set start times inside of tasks
            for task in tasks:
                # add min_window back on to starting times
                task.execution_time = task_times[task.task_id] 

                assert task.execution_time >= task.start_after
                assert task.execution_time + task.max_duration <= task.end_before

                # print 'task %s will start at %s.%s' % (task.task_id, task.execution_time.secs, task.execution_time.nsecs)                        

            tasks.sort(key=attrgetter('execution_time'))
            return True
        
        else:
            rospy.loginfo('No schedule found')
            return False


    def publish_schedule(self):
        # pub.publish(std_msgs.msg.String("foo"))
        
        exe_status = ExecutionStatus()

        current = self.execution_schedule.get_current_task()
        
        if current:
            exe_status.currently_executing = True              
            exe_status.execution_queue.append(current)

        exe_status.execution_queue.extend(self.execution_schedule.get_execution_queue())
        # print schedule
        self.schedule_publisher.publish(exe_status)

    def bound_tasks_by_start_window(self, tasks, start_after):
        bounded_tasks = []
        for task in tasks:
            # if it's still executable after the bound
            if start_after + task.max_duration <= task.end_before:
                # if we need to push it back to the bound
                if start_after > task.start_after:
                    task.start_after = start_after
                bounded_tasks.append(task)                            

        return bounded_tasks

    def try_schedule(self, additional_tasks):
        # the tasks to try and schedule
        to_schedule = []
        # add in the schedulable tasks we already have
        to_schedule.extend(self.execution_schedule.get_schedulable_tasks())
        # and the ones we have just been given
        to_schedule.extend(additional_tasks)

        # if a task is currently executing, we need to bound the proposed start windows to when we expect t
        if self.execution_schedule.get_current_task():
            bounded_tasks = self.bound_tasks_by_start_window(to_schedule, self.get_active_task_completion_time())
            if len(bounded_tasks) < len(to_schedule):
                rospy.logwarn('Dropped %s tasks which are no longer executable' % (len(to_schedule) - len(bounded_tasks)))
            to_schedule = bounded_tasks

        # reorder tasks and add execution information
        if self.call_scheduler(to_schedule):
            # if this was successful, add new tasks into execution schedule
            self.execution_schedule.add_new_tasks(additional_tasks)
            # put scheduled tasks back into execution. this will trigger a change in execution if necessary
            self.execution_schedule.set_schedule(to_schedule)
            rospy.loginfo('Added %s tasks into the schedule to get total of %s' % (len(additional_tasks), self.execution_schedule.get_execution_queue_length()))
            return True
        else:
            # previously scheduled tasks will still remain scheduled
            rospy.logwarn('Discarding %s unschedulable tasks, retaining %s' % (len(additional_tasks), self.execution_schedule.get_execution_queue_length()))
            return False

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

                self.try_schedule(unscheduled)                

            except Empty, e:
                # rospy.logdebug('No new tasks to schedule')
                pass

            self.publish_schedule()


    def execute_tasks(self):
        wait_time = 1 # 1second
        
        while not rospy.is_shutdown():           

            # print "executing thread %s" % rospy.is_shutdown()
            if(self.execution_schedule.wait_for_execution_change(wait_time)):
                next_task = self.execution_schedule.get_current_task()
                rospy.loginfo('Next task to execute: %s' % next_task.task_id)
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

