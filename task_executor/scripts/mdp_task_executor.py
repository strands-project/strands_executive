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
from copy import deepcopy
from actionlib_msgs.msg import GoalStatus

ZERO = rospy.Duration(0)

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

        # Whether or not the normal tasks need to be checked
        self.recheck_normal_tasks = False

        # how much time should we try to fill with tasks
        self.execution_window = rospy.Duration(1200)
        # and the max number of tasks to fit into this window due to MDP scaling issues
        self.batch_limit = 4

        self.time_critical_expectation_thread = Thread(target=self.process_time_critical)
        self.time_critical_expectation_thread.start()

        self.mdp_exec_thread = Thread(target=self.mdp_exec)    
        self.mdp_exec_thread.start()
        

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
                    self.recheck_normal_tasks = True

            # print len(self.normal_tasks)

    def mdp_exec_feedback(self, feedback):
        """
        Called during execution with feedback from policy execution.
        """
        with self.state_lock:
            # float64 success_prob
            # float64 expected_time
            # float64 expected_prog
            # string starting_waypoint #only this is filled atm
            # string executed_action #only this is filled atm
            # uint8 execution_status #only this is filled atm. interpert using actionlib_msgs/GoalStatus
            # string ending_waypoint
            # string next_action

            # print("Got Feedback: " + str(feedback))

            # uint8 PENDING=0
            # uint8 ACTIVE=1
            # uint8 PREEMPTED=2
            # uint8 SUCCEEDED=3
            # uint8 ABORTED=4
            # uint8 REJECTED=5
            # uint8 PREEMPTING=6
            # uint8 RECALLING=7
            # uint8 RECALLED=8
            # uint8 LOST=9

            # anything greater than preempted means that the goal is done.
            # todo: handle more detailed logging based on different statuses
            if feedback.execution_status >= GoalStatus.PREEMPTED:
                self.remove_active_task(feedback.executed_action)


    def remove_active_task(self, action_name):
        """
        Remove the indicated task from the active batch. This is based on the (valid) assumption that the action name uniquely identifies the task.
        """
        for i in range(len(self.active_batch)):
            mdp_task = self.active_batch[i]
            if mdp_task.action.name == action_name:                
                del self.active_batch[i]
                rospy.loginfo('Removing completed active task: %s. %s remaining in active batch' % (action_name, len(self.active_batch)))
                return
        rospy.logwarn('Could not find %s in active batch'  % action_name)

    def check_for_late_normal_tasks(self, now, wiggle_room):
        """ 
        Removes any normal tasks which are too late to start execution            
        """        
        while len(self.normal_tasks) > 0:

            next_normal_task = self.normal_tasks[0]
            # todo: this ignores the navigation time for this task, making task dropping more permissive than it should be. this is ok for now.
            until_next_normal_task = next_normal_task.task.end_before - now
            if until_next_normal_task < (ZERO - wiggle_room):
                rospy.loginfo('Dropping %s as %s not enough time for execution' % 
                    (next_normal_task.action.name, until_next_normal_task.to_sec()))
                self.normal_tasks = SortedCollection(self.normal_tasks[1:], key=(lambda t: t.task.end_before))                
                # todo: log dropping of task
            else:
                break


    def check_for_late_time_critical_tasks(self, now, wiggle_room):
        """ 
        Removes any time-critical tasks which are too late to start execution            
        """

        while len(self.time_critical_tasks) > 0:

            next_time_critical_task = self.time_critical_tasks[0]
            until_next_critical_task = next_time_critical_task.task.execution_time - now

            if until_next_critical_task < (ZERO - wiggle_room):
                rospy.loginfo('Dropping %s as %s not enough time for execution' % 
                    (next_time_critical_task.action.name, until_next_critical_task.to_sec()))
                self.time_critical_tasks = SortedCollection(self.time_critical_tasks[1:], key=(lambda t: t.task.execution_time))
                # todo: log dropping of task
            else:
                break


    def next_execution_batch(self):
        """
        Called when nothing is executing and another batch of tasks are required for execution.
        """
        
        with self.state_lock:
           
            now = rospy.get_rostime()

            # how late or early can a time-critical task be
            wiggle_room = rospy.Duration(180)

            self.check_for_late_time_critical_tasks(now, wiggle_room)
            self.check_for_late_normal_tasks(now, wiggle_room)                                            

            execution_window = self.execution_window

            # now see how much time is available until the next time critical task
            if len(self.time_critical_tasks) > 0:
                next_time_critical_task = self.time_critical_tasks[0]
                until_next_critical_task = next_time_critical_task.task.execution_time - now
                rospy.loginfo('Time until next critical task %s' % until_next_critical_task.to_sec())

                if until_next_critical_task < execution_window:
                    execution_window = until_next_critical_task


            # if we're close to a time critical task, then do that
            if len(self.time_critical_tasks) > 0 and rostime_close(now, next_time_critical_task.task.execution_time, delta = wiggle_room):                               
               
                self.active_batch = [next_time_critical_task]
                self.time_critical_tasks = SortedCollection(self.time_critical_tasks[1:], key=(lambda t: t.task.execution_time))
                            
                mdp_spec = MdpDomainSpec()
                mdp_spec.vars.append(next_time_critical_task.state_var)
                mdp_spec.actions.append(next_time_critical_task.action)                    
                mdp_spec.ltl_task = '(F %s=1)'% next_time_critical_task.state_var.name                
                mdp_goal = ExecutePolicyExtendedGoal(spec = mdp_spec)
                rospy.loginfo('executing time-critical task: %s' % mdp_goal.spec.ltl_task)
                self.mdp_exec_queue.put(mdp_goal)

            # else see what we can squeeze into available time
            elif self.recheck_normal_tasks:

                rospy.loginfo('Checking for normal tasks to fit into available time: %s' % execution_window.to_sec())

                # create mdp task batch to fit into available time
                # 
                # this checks expected time after adding each task to the batch

                if len(self.normal_tasks) == 0:
                    rospy.loginfo('No normal tasks remaining')
                    self.recheck_normal_tasks = False
                else:
                    batch_size = 0
                    ltl_task = ''
                    request = GetGuaranteesForCoSafeTaskRequest()

                    mdp_estimates = rospy.ServiceProxy('mdp_plan_exec/get_guarantees_for_co_safe_task', GetGuaranteesForCoSafeTask)
                    mdp_estimates.wait_for_service()
                    last_successful_spec = None

                    mdp_spec = MdpDomainSpec()

                    for mdp_task in self.normal_tasks[:self.batch_limit]:                
                        if batch_size == self.batch_limit:
                            break
                        
                        mdp_spec.vars.append(mdp_task.state_var)
                        mdp_spec.actions.append(mdp_task.action)                    
                        ltl_task += '(F %s=1) & '% mdp_task.state_var.name                
                        mdp_spec.ltl_task = ltl_task[:-3]
                        request.spec = mdp_spec
                        service_response = mdp_estimates(request)
                        expected_policy_duration = service_response.expected_times[service_response.initial_waypoints.index(self.get_topological_node())]
                        
                        if expected_policy_duration > execution_window:                        
                            print 'too long policy duration for %s: %s' % (mdp_spec.ltl_task, expected_policy_duration.to_sec())                    
                            break
                        else:
                            print 'acceptable policy duration for %s: %s' % (mdp_spec.ltl_task, expected_policy_duration.to_sec())
                            last_successful_spec = deepcopy(mdp_spec)

                        batch_size += 1

                    # if we found tasks to fit into the time available
                    if batch_size > 0:

                        self.active_batch = self.normal_tasks[:batch_size]
                        self.normal_tasks = SortedCollection(self.normal_tasks[batch_size:], key=(lambda t: t.task.end_before))                
                        
                        mdp_goal = ExecutePolicyExtendedGoal(spec = last_successful_spec)
                        rospy.loginfo('Executing normal batch: %s' % mdp_goal.spec.ltl_task)
                        self.mdp_exec_queue.put(mdp_goal)
                    
                    # if we couldn't fit a batch in, but there are normal tasks available (we checked above)
                    else:
                        # if the first available task won't fit into the available execution time window, and this is the max possible, then increase the window size accordingly
                        if execution_window == self.execution_window:                        
                            # for now just increase to the expected time of last tested policy
                            self.execution_window = expected_policy_duration 
                            rospy.loginfo('Extending default execution windown to %s' % self.execution_window.to_sec())
                        
                        # if we get here then we can't fit the first available task into the time before the first time-critical task
                        else:
                            # the basic thing here is not to recheck the normal tasks until after the next time-critical execution or until new normal tasks are added (which could be potentially earlier/shorter)
                            self.recheck_normal_tasks = False
                            # todo: we could also try some optimisation to fit in a task other than the first available normal one
            else:
                rospy.logdebug('No need to recheck normal tasks')



    def mdp_exec(self):
        """
        This is the main loop of the executor. It checks for the next goal to execute. 
        If there's nothing to execute then it calls next_execution_batch to check for available tasks.
        """
        while not rospy.is_shutdown():

            if self.executing:            
                try:
                    mdp_goal = self.mdp_exec_queue.get(timeout = 1)

                    print 'got a goal from queue'

                    # execution status could have changed during the poll of the queue
                    if self.executing:            

                        print 'and executing'

                        with self.state_lock:
                            self.mdp_exec_client = actionlib.SimpleActionClient('mdp_plan_exec/execute_policy_extended', ExecutePolicyExtendedAction)
                            self.mdp_exec_client.wait_for_server()
                            self.mdp_exec_client.send_goal(mdp_goal, feedback_cb = self.mdp_exec_feedback)

                        self.mdp_exec_client.wait_for_result()
                        print 'wait completed'
                        print 'state'
                        print self.mdp_exec_client.get_state()
                        print 'result'
                        print self.mdp_exec_client.get_result()


                        with self.state_lock:
                            # make sure this can't be used now execution is complete
                            self.mdp_exec_client = None
                            # whatever happened or was executed, we should now recheck the available normal tasks
                            self.recheck_normal_tasks = True
                except Empty, e:
                    pass

                # state of execution could have changed since the last check
                if self.executing:            
                    self.next_execution_batch()
            else:
                rospy.sleep(1)
            

    def process_time_critical(self):
        """
        Poll queue of new time-critical tasks and then add navigation duration to them.
        """

        while not rospy.is_shutdown():
            
            try:
                mdp_task = self.new_time_critical_tasks.get(timeout = 1)

                rospy.wait_for_service('mdp_plan_exec/get_guarantees_for_co_safe_task')
                mdp_estimates = rospy.ServiceProxy('mdp_plan_exec/get_guarantees_for_co_safe_task', GetGuaranteesForCoSafeTask)

                with self.state_lock:
                    rospy.loginfo('Processing new time-critical task: %s' % mdp_task.action.name)                

                    # rospy.loginfo('Time critical tasks at start of process_time_critical:')
                    # for m in self.time_critical_tasks:
                    #     rospy.loginfo(m.task.task_id)
                    #     rospy.loginfo(m.action.name)

                    request = GetGuaranteesForCoSafeTaskRequest()
                    request.spec.ltl_task = '(F "%s")' % mdp_task.task.start_node_id
                    response = mdp_estimates(request)

                    # response gives expected time from all waypoints
                    # as we can't assume we're just doing it from where we are
                    # check from here plus the 

                    potential_start_points = set([self.get_topological_node()]) 
                
                    tasks_to_start_from = [m.task for m in self.normal_tasks]
                    tasks_to_start_from += [m.task for m in self.active_batch]

                    for task in tasks_to_start_from:
                        if len(task.end_node_id) == 0:
                            if len(task.start_node_id) != 0:
                                potential_start_points.add(task.start_node_id)
                        else:
                            potential_start_points.add(task.end_node_id)

                    # print 'checking from ', potential_start_points
                    duration_total = rospy.Duration(0)
                    for wp in potential_start_points:
                        duration_total += response.expected_times[response.initial_waypoints.index(wp)]
                    
                    expected_nav_time = duration_total / len(potential_start_points)
                    print '"expected" nav time: ', expected_nav_time.to_sec()

                    mdp_task.task.execution_time = mdp_task.task.start_after - expected_nav_time

                    self.time_critical_tasks.insert(mdp_task)

                    rospy.loginfo('%s time-critical tasks' % len(self.time_critical_tasks))

                    # rospy.loginfo('Time critical tasks at end of process_time_critical:')
                    # for m in self.time_critical_tasks:
                    #     rospy.loginfo(m.task.task_id)
                    #     rospy.loginfo(m.action.name)

            except Empty, e:
                pass



        

    def start_execution(self):
        """ Called when overall execution should  (re)start """
        rospy.loginfo('(Re-)starting execution')        

    def pause_execution(self):
        """ Called when overall execution should pause. This is called *before* self.executing is set to False. """
        with self.state_lock:            

            # this is done by the super class *after* pause_execution completes, but we need to make sure that it is done before this lock is released to make sure execution does not continue after policy execution preemption
            self.executing = False

            # keep count for later reporting
            active_count = len(self.active_batch)

            # for each task remaining in the active batch, put it back into the right list
            for mdp_task in self.active_batch:                    
                if mdp_task.task.start_after == mdp_task.task.end_before:                        
                    self.time_critical_tasks.insert(mdp_task)
                else:
                    self.normal_tasks.insert(mdp_task)                                                

            # empty the active batch. this might mean some feedback misses the update
            # the consequence is that the task was completed but we preempted before receiving the update, 
            # this means the task will be executed again, but there's no easy way around this
            self.active_batch = []

            # If the client is not None then there is execution going on. the active batch could be empty if we've just caught the tail end of execution
            # 
            # Also there could be tasks in the active batch without an action client existing. as the client is created slightly later
            if self.mdp_exec_client is not None:

                # preempt the action server
                self.mdp_exec_client.cancel_all_goals()
                rospy.loginfo('Cancelling policy execution')

            rospy.loginfo('Execution paused with %s tasks in the active batch' % active_count)


    def task_demanded(self, demanded_task, currently_active_task):
        """ Called when a task is demanded. self.active_task is the demanded task (and is being executed) and previously_active_task was the task that was being executed (which could be None) """
        pass

    def cancel_active_task(self):
        """ 
        Called to cancel the task which is currently executing.
        If something is being executed we handle this by simply pausing and restarting execution.
        pause_execution is often called before this. (this is always the case currently)
        """
        if self.executing:
            self.pause_execution()
            self.executing = True

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

