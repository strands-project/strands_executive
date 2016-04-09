#!/usr/bin/env python

from __future__ import with_statement 
import rospy
from Queue import Queue, Empty
from strands_executive_msgs.msg import Task, ExecutionStatus, DurationMatrix, DurationList, ExecutePolicyExtendedAction, ExecutePolicyExtendedFeedback, ExecutePolicyExtendedGoal, MdpStateVar, StringIntPair, StringTriple, MdpAction, MdpActionOutcome, MdpDomainSpec
from strands_executive_msgs.srv import GetGuaranteesForCoSafeTask, GetGuaranteesForCoSafeTaskRequest
from task_executor.base_executor import BaseTaskExecutor
from threading import Thread
from task_executor.execution_schedule import ExecutionSchedule
from operator import attrgetter
import copy
from math import floor
import threading
import actionlib
from task_executor.SortedCollection import SortedCollection
from task_executor.utils import rostime_to_python, rostime_close
from dateutil.tz import tzlocal


class MDPTask(object):
    """
    Class to store task and mdp related stuff together.
    """

    def __init__(self, task, state_var, action):
        self.task = task
        self.state_var = state_var
        self.action = action




class MDPTaskExecutor(BaseTaskExecutor):
    """

    Executor which receives Task objects and converts them into MdpActions and manages their execution.

    This distinguishes between three different types of tasks:
     a) On demand tasks, which should be executed immediately.
     b) Time-critical tasks, which should be executed as close to their start time as possible
     c) Normal tasks, which should preferably (but not necessarily) be executed within their time window as possible    

    On demand tasks are added by the demand task service. The other types are added by the add task service. Time critical tasks are identified by having the same start and end time.

    This executor respects task priorities and interruptibility in as far as tasks which declare themselves as uninterruptible will not be interrupted by a same or lower priority on-demand task, and no uninterruptible task will be cancelled due to a timeout.

    The clear schedule service cancels all execution (regardless of uninterruptibility state) removes all tasks from the executor.

    The executor publishes a schedule which is an ordering over tasks indicating the approximate order they will be considered for execution.

    Normal tasks are sent to MDP execution in batches. These batches are limited to a configurable size (rosparam ~mdp_batch_size). On-demand and time critical tasks always have a batch size of one.

    """

    def __init__(self):
        # init node first, must be done before call to super init for service advertising to work
        rospy.init_node("task_executor", log_level=rospy.INFO)

        # init superclasses
        super( MDPTaskExecutor, self ).__init__()

        self.new_time_critical_tasks = Queue()

        # collection of MDPTasks sorted by deadline
        self.normal_tasks = SortedCollection(key=(lambda t: t.task.end_before))
        self.time_critical_tasks = SortedCollection(key=(lambda t: t.task.execution_time))


        self.state_lock = threading.Lock()
        self.mdp_exec_client = None
        self.active_batch = []
        self.mdp_exec_queue = Queue()

        self.mdp_exec_thread = Thread(target=self.mdp_exec)    
        self.mdp_exec_thread.start()
        
        self.time_critical_expectation_thread = Thread(target=self.process_time_critical)
        self.time_critical_expectation_thread.start()

        self.advertise_services()
        self.tz = tzlocal()
        

    def _convert_task_to_mdp_action(self, task):
        """ Converts a Task to a MdpAction.
            returns a task, state var, action triple
        """

        # print task


        action_name = 'n'+ str(task.task_id) + '_' + task.action + '_at_' + task.start_node_id  
        state_var_name = 'executed_' + action_name

        state_var = MdpStateVar(name = state_var_name,
                init_val = 0, min_range = 0,
                max_range = 1)

        outcome=MdpActionOutcome(probability = 1.0,
                post_conds = [StringIntPair(string_data = state_var_name, int_data = 1)],
                duration_probs = [1.0],
                durations = [task.max_duration.to_sec()])

        action = MdpAction(name=action_name, 
                 action_server=task.action, 
                 waypoints=[task.start_node_id],
                 pre_conds=[StringIntPair(string_data=state_var_name, int_data=0)],
                 outcomes=[outcome])

        action.arguments = task.arguments

        # print state_var
        # print action

        return MDPTask(task, state_var, action)

    def add_tasks(self, tasks):
        """ Called with new tasks for the executor """

        with self.state_lock:
            
            for task in tasks:
                # print task
                mdp_task = self._convert_task_to_mdp_action(task)
                if task.start_after == task.end_before:
                    # queue up the time critical tasks for processing
                    self.new_time_critical_tasks.put(mdp_task)
                else:
                    self.normal_tasks.insert(mdp_task)

            print len(self.normal_tasks)

    def mdp_exec_feedback(self, feedback):
        print("Got Feedback: " + str(feedback))


    def next_execution_batch(self):
        """
        Called when nothing is executing and another batch of tasks are required for execution.
        """
        
        with self.state_lock:
            # are we in the window for a time-critical task?
            # 

            now = rospy.get_rostime()

            # how much time is available for execution from now
            available_time = rospy.Duration(3600)

            if len(self.time_critical_tasks) > 0:
                next_time_critical_task = self.time_critical_tasks[0]

                # print 'now: ', rostime_to_python(now, tz=self.tz)
                # print 'start after: ', rostime_to_python(next_time_critical_task.task.start_after, tz=self.tz)
                # print 'first critical: ', rostime_to_python(next_time_critical_task.task.execution_time, tz=self.tz)

                until_next_critical_task = next_time_critical_task.task.execution_time - now
                print 'time to next critical task %s' % until_next_critical_task.to_sec()
                if until_next_critical_task < available_time:
                    available_time = until_next_critical_task

            print 'available time for normal tasks %s' % available_time.to_sec()

            # create mdp task, checking time at each point

            batch_limit = 5
            batch_size = 0
            ltl_task = ''
            mdp_spec = MdpDomainSpec()
            request = GetGuaranteesForCoSafeTaskRequest()

            mdp_estimates = rospy.ServiceProxy('mdp_plan_exec/get_guarantees_for_co_safe_task', GetGuaranteesForCoSafeTask)
            mdp_estimates.wait_for_service()

            for mdp_task in self.normal_tasks[:batch_limit]:                
                if batch_size == batch_limit:
                    break

                mdp_spec.vars.append(mdp_task.state_var)
                mdp_spec.actions.append(mdp_task.action)                    
                ltl_task += '(F %s=1) & '% mdp_task.state_var.name                
                mdp_spec.ltl_task = ltl_task[:-3]
                request.spec = mdp_spec
                service_response = mdp_estimates(request)
                expected_policy_duration = service_response.travel_times[service_response.initial_waypoints.index(self.get_topological_node())]
                print 'policy duration: ', expected_policy_duration.to_sec()
                
                if expected_policy_duration > available_time:
                    break

                batch_size += 1

            print 'number of normal tasks: ', batch_size            

            if batch_size > 0:
                self.active_batch = self.normal_tasks[:batch_size]
                self.normal_tasks = SortedCollection(self.normal_tasks[batch_size:], key=(lambda t: t.task.end_before))                
                
                mdp_goal = ExecutePolicyExtendedGoal()
                mdp_spec.ltl_task = ltl_task[:-3]
                mdp_goal.spec = mdp_spec
                self.mdp_exec_queue.put(mdp_goal)
                
            elif len(self.time_critical_tasks) and rostime_close(now, next_time_critical_task.task.execution_time):
                print 'going for time critical'
                self.active_batch = [next_time_critical_task]
                self.time_critical_tasks = SortedCollection(self.time_critical_tasks[1:], key=(lambda t: t.task.execution_time))

                mdp_goal = ExecutePolicyExtendedGoal()
                
                mdp_spec = MdpDomainSpec()
                mdp_spec.vars.append(next_time_critical_task.state_var)
                mdp_spec.actions.append(next_time_critical_task.action)                    
                mdp_spec.ltl_task = '(F %s=1)'% next_time_critical_task.state_var.name                
                mdp_goal.spec = mdp_spec    
                self.mdp_exec_queue.put(mdp_goal)


    def mdp_exec(self):

        while not rospy.is_shutdown():
            
            try:
                mdp_goal = self.mdp_exec_queue.get(timeout = 1)
                self.mdp_exec_client = actionlib.SimpleActionClient('mdp_plan_exec/execute_policy_extended', ExecutePolicyExtendedAction)
                self.mdp_exec_client.wait_for_server()
                self.mdp_exec_client.send_goal(mdp_goal, feedback_cb = self.mdp_exec_feedback)
                self.mdp_exec_client.wait_for_result()
            except Empty, e:
                pass

            self.next_execution_batch()
            
    def process_time_critical(self):

        while not rospy.is_shutdown():
            
            try:
                mdp_task = self.new_time_critical_tasks.get(timeout = 1)
                # mdp_estimates = rospy.ServiceProxy('mdp_plan_exec/get_guarantees_for_co_safe_task', GetGuaranteesForCoSafeTask)
                # request = GetGuaranteesForCoSafeTaskRequest()
                # faked nav time
                expected_nav_time = rospy.Duration(120)
                mdp_task.task.execution_time = mdp_task.task.start_after - expected_nav_time
                with self.state_lock:
                    self.time_critical_tasks.insert(mdp_task)

            except Empty, e:
                pass



        

    def start_execution(self):
        """ Called when overall execution should  (re)start """
        print 'starting'
        self.next_execution_batch()

    def pause_execution(self):
        """ Called when overall execution should pause """
        pass

    def task_complete(self, task):
        """ Called when the given task has completed execution """
        pass

    def task_succeeded(self, task):
        """ Called when the given task has completed execution successfully """
        self.task_complete(task)

    def task_failed(self, task):
        """ Called when the given task has completed execution but failed """
        self.task_complete(task)


    def task_demanded(self, demanded_task, currently_active_task):
        """ Called when a task is demanded. self.active_task is the demanded task (and is being executed) and previously_active_task was the task that was being executed (which could be None) """
        pass

    def cancel_active_task(self):
        """ Called to cancel the task which is currently executing """
        pass

    def cancel_task(self, task_id):
        """ Called when a request is received to cancel a task. The currently executing one is checked elsewhere. """
        return False

    def clear_schedule(self):
        """ Called to clear all tasks from schedule, with the exception of the currently executing one. """
        pass



if __name__ == '__main__':
    executor = MDPTaskExecutor()        
    rospy.spin()


# create a schedule class which handles blocking until execution and manages the various changes

