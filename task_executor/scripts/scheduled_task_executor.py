#!/usr/bin/env python

import rospy
from Queue import Queue, Empty
from strands_executive_msgs.msg import Task, ExecutionStatus, DurationMatrix, DurationList
from strands_executive_msgs.srv import GetSchedule
from task_executor.sm_base_executor import AbstractTaskExecutor
from threading import Thread
from task_executor.execution_schedule import ExecutionSchedule
from operator import attrgetter
import copy

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
        self.schedule_publisher = rospy.Publisher('/current_schedule', ExecutionStatus, queue_size=1)

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
            # if no end node, set it to the same as the start
            if task.end_node_id == '':
                task.end_node_id = task.start_node_id 

        for task in tasks:
            self.unscheduled_tasks.put(task)

    def task_complete(self, task):
        """ Called when the given task has completed execution """
        # pass signal to schedule
        self.execution_schedule.task_complete(task) 

    def task_demanded(self, demanded_task, currently_active_task):
        """ Called when a task is demanded. self.active_task is the demanded task (and is being executed) and previously_active_task was the task that was being executed (which could be None) """

        if demanded_task.end_node_id == '':
            demanded_task.end_node_id = demanded_task.start_node_id 

        rospy.loginfo('Task %s, %s demanded from scheduler' % (demanded_task.task_id, demanded_task.action))

        # save the tasks we had previously
        previously_scheduled = self.execution_schedule.get_schedulable_tasks()

        # clear all tasks that were previously scheduled
        self.execution_schedule.clear_schedule()

        # wait until the demanded task has been taken off for execution, otherwise we run into problems with get_schedulable_tasks
        # cancellation should block until cancelled 
        # while self.execution_schedule.get_current_task() is not None and not rospy.is_shutdown():
        #     rospy.sleep(1)
        #     rospy.loginfo('Waiting for previous task to finish')

        # try to schedule them back in 
        self.execution_schedule.add_new_tasks([demanded_task])
        # put scheduled tasks back into execution. this will trigger a change in execution if necessary
        self.execution_schedule.set_schedule([demanded_task])


        # now try to put the other tasks back in
        if len(previously_scheduled) > 0:
            success, added = self.try_schedule(previously_scheduled)
            if success:
                rospy.loginfo('Was able to reinstate %s/%s tasks after demand' % (len(added), len(previously_scheduled)))
                if currently_active_task != None:
                    success, added = self.try_schedule([currently_active_task])
                    if success and len(added) > 0:
                        rospy.loginfo('Was also able to reinstate previously active task after demand')
                    else:
                        rospy.loginfo('Was not able to reinstate previously active task after demand (but other tasks ok)')
            else:
                rospy.loginfo('Was NOT able to reinstate tasks after demand')



    def get_duration(self, start, end):
        return 1.0

    def get_duration_matrix(self, tasks):
        """
        Creates the matrix of durations between waypoints needed as input to the scheuler.
        Output is a DurationMatrix encoding  duration[i][j] where this is the duration expected for travelling between the end of the ith task in tasks and the start of the jth element.
        """
        # first populate sets of start and end locations for moves between tasks, these are the end and start of tasks repsectively
        start_nodes = set([task.end_node_id for task in tasks])
        end_nodes = set([task.start_node_id for task in tasks])

        # next get the costs for each element of the cross product
        durations = dict()
        for start in start_nodes:
            durations[start] = dict()
            for end in end_nodes:
                durations[start][end] = self.get_duration(start, end)


        # now populate the DurationMatrix object
        dm = DurationMatrix()
        for task_i in tasks:
            dm.durations.append(DurationList())
            for task_j in tasks:
                dm.durations[-1].durations.append(durations[task_i.end_node_id][task_j.start_node_id])

        return dm




    def call_scheduler(self, tasks, earliest_start, current_id=0):
        """ 
        
        Calls scheduler. Reorders the list of tasks in execution order with their execution times set. 
        
        """
        resp = self.schedule_srv(tasks, earliest_start, current_id)

        rospy.loginfo(resp)

        if len(resp.task_order) > 0:

            # add start times to a dictionary for fast lookup
            task_times = {}
            for (task_id, start_time) in zip(resp.task_order, resp.execution_times):
                print task_id, start_time
                task_times[task_id] = start_time

            # set start times inside of tasks
            for task in tasks:
                # add min_window back on to starting times
                print task.task_id, task_times[task.task_id] 
                task.execution_time = task_times[task.task_id] 

                # taking out as rescheduling demanded tasks have an issue here I think
                # assert task.execution_time >= task.start_after
                # assert task.execution_time + task.max_duration <= task.end_before

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
        dropped_tasks = []
        for task in tasks:
            # if it's still executable after the bound
            if start_after + task.max_duration <= task.end_before:
                # 
                # This is no longer done because it means that execution also has to work with a bounded start_after which may actually lead to unnecessary delays
                # 
                # # if we need to push it back to the bound
                # if start_after > task.start_after:
                    # # rospy.loginfo('pushing boundary')
                    # task.start_after = start_after
                bounded_tasks.append(task)                            
            else:
                dropped_tasks.append(task)                            

        return bounded_tasks, dropped_tasks

    def try_schedule(self, additional_tasks):
        
        # this is the lower bound on future execution time
        lower_bound = rospy.get_rostime()
        if self.execution_schedule.get_current_task() is not None:
            lower_bound = self.get_active_task_completion_time()
            rospy.logwarn('lower bound %s' % lower_bound)

        # bound additional tasks bu this bound
        bounded_tasks, dropped_tasks = self.bound_tasks_by_start_window(additional_tasks, lower_bound)
        if len(bounded_tasks) < len(additional_tasks):
            rospy.logwarn('Dropped %s additional tasks which are no longer executable' % (len(additional_tasks) - len(bounded_tasks)))
        additional_tasks = bounded_tasks


        # these are previoulsy scheduled tasks which we must now reschedule
        schedulable_tasks = self.execution_schedule.get_schedulable_tasks()
               
        bounded_tasks, dropped_tasks = self.bound_tasks_by_start_window(schedulable_tasks, lower_bound)
        if len(bounded_tasks) < len(schedulable_tasks):
            rospy.logwarn('Dropped %s existing tasks which are no longer executable' % (len(schedulable_tasks) - len(bounded_tasks)))
            # have to remove these from schedule too, although this assumes successful scheduling
            # TODO: what if scheduling is not successful?
            self.execution_schedule.remove_tasks(dropped_tasks)
        schedulable_tasks = bounded_tasks
        
     
        # the tasks to try and schedule
        to_schedule = []
        # add in the schedulable tasks we already have
        to_schedule.extend(schedulable_tasks)
        # and the ones we have just been given
        to_schedule.extend(additional_tasks)                

        # if we are currently executing something, including this for the start of the schedule too
        current_id = 0
        current_task = copy.deepcopy(self.execution_schedule.get_current_task())
        if current_task is not None:
            current_id = current_task.task_id
            rospy.loginfo('Including current task %s in scheduling problem' % current_id)
            # assuming this executed when requested (may not be!) update how much time is left
            current_task.max_duration = rospy.Time(1)
            current_task.end_before = lower_bound
            current_task.start_after = lower_bound - rospy.Time(2)
            to_schedule.append(current_task)

        # reorder tasks and add execution information
        if self.call_scheduler(to_schedule, lower_bound, current_id):
            # if this was successful, add new tasks into execution schedule
            self.execution_schedule.add_new_tasks(additional_tasks)

            # and remove the current task from the new schedule
            to_schedule = [t for t in to_schedule if t.task_id != current_id]

            # put scheduled tasks back into execution. this will trigger a change in execution if necessary
            self.execution_schedule.set_schedule(to_schedule)

            rospy.loginfo('Added %s tasks into the schedule to get total of %s' % (len(additional_tasks), self.execution_schedule.get_execution_queue_length()))
            return True, additional_tasks
        else:
            # previously scheduled tasks will still remain scheduled
            rospy.logwarn('Discarding %s unschedulable tasks, retaining %s' % (len(additional_tasks), self.execution_schedule.get_execution_queue_length()))
            return False, []

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
                if next_task is not None:
                    self.execute_task(next_task)                
                else:
                    rospy.logwarn('Next task was None')


    def cancel_task(self, task_id):
        """ Called when a request is received to cancel a task. The currently executing one is checked elsewhere. """
        if self.execution_schedule.remove_task_with_id(task_id):
            # reschedule after a successful removal
            self.try_schedule([])
            return True
        else:
            return False

    def clear_schedule(self):
        """ Called to clear all tasks from schedule, with the exception of the currently executing one. """
        self.execution_schedule.clear_schedule()



if __name__ == '__main__':
    executor = ScheduledTaskExecutor()        
    rospy.spin()


# create a schedule class which handles blocking until execution and manages the various changes

