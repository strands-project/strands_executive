#!/usr/bin/env python

from __future__ import with_statement 
import rospy
from Queue import Queue, Empty
from strands_executive_msgs.msg import Task, ExecutionStatus, DurationMatrix, DurationList, ExecutePolicyExtendedAction, ExecutePolicyExtendedFeedback, ExecutePolicyExtendedGoal, MdpStateVar, StringIntPair, StringTriple, MdpAction, MdpActionOutcome, MdpDomainSpec
from task_executor.base_executor import BaseTaskExecutor
from threading import Thread
from task_executor.execution_schedule import ExecutionSchedule
from operator import attrgetter
import copy
from math import floor
import threading
import actionlib
from task_executor.SortedCollection import SortedCollection


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
        self.state_lock = threading.Lock()
        self.mdp_exec_client = None
        self.active_batch = []
        self.mdp_exec_queue = Queue()

        self.mdp_exec_thread = Thread(target=self.mdp_exec)    
        self.mdp_exec_thread.start()
        
        self.advertise_services()

        

    def _convert_task_to_mdp_action(self, task):
        """ Converts a Task to a MdpAction.
            returns a task, state var, action triple
        """

        # print task


        action_name = task.action + '_at_' + task.start_node_id
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
            # if not take the next n tasks
            n = 5
            self.active_batch = self.normal_tasks[:n]
            if len(self.active_batch) > 0:
                self.normal_tasks = SortedCollection(self.normal_tasks[n:], key=(lambda t: t.task.end_before))                
                
                mdp_goal = ExecutePolicyExtendedGoal()
                mdp_spec = MdpDomainSpec()
                ltl_task = ''
                for mdp_task in self.active_batch:
                    mdp_spec.vars.append(mdp_task.state_var)
                    mdp_spec.actions.append(mdp_task.action)
                    
                    ltl_task += '(F %s=1) & '% mdp_task.state_var.name
                
                mdp_spec.ltl_task = ltl_task[:-3]
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

