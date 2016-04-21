#!/usr/bin/env python

from __future__ import with_statement 
import rospy
from Queue import Queue, Empty
from strands_executive_msgs.msg import Task, ExecutionStatus, DurationMatrix, DurationList, ExecutePolicyExtendedAction, ExecutePolicyExtendedFeedback, ExecutePolicyExtendedGoal, MdpStateVar, StringIntPair, StringTriple, MdpAction, MdpActionOutcome, MdpDomainSpec, TaskEvent
from strands_executive_msgs.srv import GetGuaranteesForCoSafeTask, GetGuaranteesForCoSafeTaskRequest
from task_executor.base_executor import BaseTaskExecutor
from threading import Thread, Condition
from task_executor.execution_schedule import ExecutionSchedule
from operator import attrgetter
import copy
from math import floor
import threading
import actionlib
from task_executor.SortedCollection import SortedCollection
from task_executor.utils import rostime_to_python, rostime_close
from dateutil.tz import tzlocal
from copy import copy, deepcopy
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
        self.set_active_batch([])
        # only ever allow one batch in the execution queue. If this restriction is removed then demanding won't work immediately
        self.mdp_exec_queue = Queue(maxsize = 1)

        # Whether or not the normal tasks need to be checked
        self.recheck_normal_tasks = False

        # how much time should we try to fill with tasks. this is the default and will be extended if necessary
        self.execution_window = rospy.Duration(1200)
        
        # and the max number of tasks to fit into this window due to MDP scaling issues
        self.batch_limit = 6

        self.time_critical_expectation_thread = Thread(target=self._process_time_critical)
        self.time_critical_expectation_thread.start()

        self.mdp_exec_thread = Thread(target=self.mdp_exec)    
        self.mdp_exec_thread.start()
        
        # topic on which current schedule is broadcast
        self.schedule_publisher = rospy.Publisher('current_schedule', ExecutionStatus, latch = True, queue_size = 1)

        self.update_schedule_condition = Condition()
        self.schedule_publish_thread = Thread(target=self.publish_schedule)
        self.schedule_publish_thread.start()

        self.advertise_services()
        self.tz = tzlocal()
        

    def _convert_task_to_mdp_action(self, task):
        """ Converts a Task to a MdpAction.
            returns a task, state var, action triple
        """

        action_name = 'n'+ str(task.task_id) + '_' + task.action + '_at_' + task.start_node_id  
        # make sure there is nothing to make PRISM cry
        action_name = action_name.replace('/','_')

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

        if self.recheck_normal_tasks:
            self.republish_schedule()


    def goal_status_to_task_status(self, goal_status):
        if goal_status == GoalStatus.PREEMPTED:
            return TaskEvent.TASK_PREEMPTED
        elif goal_status == GoalStatus.SUCCEEDED:
            return TaskEvent.TASK_SUCCEEDED
        elif goal_status == GoalStatus.ACTIVE:
            return TaskEvent.TASK_FAILED
        else:
            if goal_status != GoalStatus.ABORTED:
                rospy.logwarn('Unknown conversion to TaskStatus for %s' % GoalStatus.to_string(goal_status))
            return TaskEvent.TASK_FAILED


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

                        
            rospy.loginfo('%s received feedback %s' % (feedback.executed_action, GoalStatus.to_string(feedback.execution_status)))
            
            # if feedback.execution_status >= GoalStatus.PREEMPTED:
            # we don't need to check this status as we only recieve this feedback in the terminal states of the mdp, so this action is done regarless
            # todo: PREEMPTED means the action started but was cancelled during the execution if the action server
            # todo: ACTIVE means the action started but was cancelled during the execution if the action server but didn't prempt
            # todo: show we allow these to be re-added to execution queue? currently preemption signals that a task has been permanently removed
            # todo: if added back to normal tasks it will almost certainly be re-executed immediately as it's at the current location, causing a loop

            self.remove_active_task(feedback.executed_action, self.goal_status_to_task_status(feedback.execution_status))
        self.republish_schedule()

    def remove_active_task(self, action_name, task_status):
        """
        Remove the indicated task from the active batch. This is based on the (valid) assumption that the action name uniquely identifies the task.
        """
        for i in range(len(self.active_batch)):
            mdp_task = self.active_batch[i]
            if mdp_task.action.name == action_name:                
                del self.active_batch[i]
                del self.active_tasks[i]
                log_string = 'Removing completed active task: %s. %s remaining in active batch' % (action_name, len(self.active_batch))
                rospy.loginfo(log_string)
                self.log_task_event(mdp_task.task, task_status, rospy.get_rostime(), description = log_string)        
                return
        rospy.logwarn('Could not find %s in active batch'  % action_name)

    def check_for_late_normal_tasks(self, now, wiggle_room):
        """ 
        Removes any normal tasks which are too late to start execution            
        """       
        dropped = False

        while len(self.normal_tasks) > 0:

            next_normal_task = self.normal_tasks[0]
            # todo: this ignores the navigation time for this task, making task dropping more permissive than it should be. this is ok for now.
            until_next_normal_task = next_normal_task.task.end_before - now
            if until_next_normal_task < (ZERO - wiggle_room):
                log_string = 'Dropping normal task %s as %s not enough time for execution' % (next_normal_task.action.name, until_next_normal_task.to_sec())
                rospy.loginfo(log_string)
                self.normal_tasks = SortedCollection(self.normal_tasks[1:], key=(lambda t: t.task.end_before))                
                self.log_task_event(next_normal_task.task, TaskEvent.DROPPED, now, description = log_string)        
                dropped = True
            else:
                break

        return dropped


    def check_for_late_time_critical_tasks(self, now, wiggle_room):
        """ 
        Removes any time-critical tasks which are too late to start execution            
        """
        dropped = False
        while len(self.time_critical_tasks) > 0:

            next_time_critical_task = self.time_critical_tasks[0]
            until_next_critical_task = next_time_critical_task.task.execution_time - now

            if until_next_critical_task < (ZERO - wiggle_room):
                log_string = 'Dropping time-critical task %s as %s not enough time for execution' % (next_time_critical_task.action.name, until_next_critical_task.to_sec())
                rospy.loginfo(log_string)
                self.time_critical_tasks = SortedCollection(self.time_critical_tasks[1:], key=(lambda t: t.task.execution_time))
                self.log_task_event(next_time_critical_task.task, TaskEvent.DROPPED, now, description = log_string)        
                dropped = True
            else:
                break
        return dropped

    def _mdp_single_task_to_goal(self, mdp_task):
        mdp_spec = MdpDomainSpec()
        mdp_spec.vars.append(mdp_task.state_var)
        mdp_spec.actions.append(mdp_task.action)                    
        mdp_spec.ltl_task = '(F %s=1)'% mdp_task.state_var.name                
        return ExecutePolicyExtendedGoal(spec = mdp_spec)
             

    def _mdp_tasks_to_spec(self, mdp_tasks):
        """
        Take a collection of MDPTask objects and produce an MdpDomainSpec from them.
        """
        mdp_spec = MdpDomainSpec()
        mdp_spec.ltl_task = '(F ('
        for mdp_task in mdp_tasks:

            mdp_spec.vars.append(mdp_task.state_var)
            mdp_spec.actions.append(mdp_task.action)
            mdp_spec.ltl_task += '%s=1 & ' % mdp_task.state_var.name                
        
        mdp_spec.ltl_task = mdp_spec.ltl_task[:-3]
        mdp_spec.ltl_task += '))'
        return mdp_spec

            

    def _drop_out_of_time_tasks(self, now, wiggle_room):
        """
        Drop any normal or time-critical task when their time windows have been exceeded.
        """
        dropped = self.check_for_late_time_critical_tasks(now, wiggle_room)
        dropped = dropped or self.check_for_late_normal_tasks(now, wiggle_room)                                            

        if dropped:
            self.republish_schedule()

    def _get_guarantees_for_batch(self, task_batch, estimates_service = None):
        if estimates_service == None:
            estimates_service = rospy.ServiceProxy('mdp_plan_exec/get_guarantees_for_co_safe_task', GetGuaranteesForCoSafeTask)
            estimates_service.wait_for_service()

        spec = self._mdp_tasks_to_spec(task_batch)
        request = GetGuaranteesForCoSafeTaskRequest(spec = spec, initial_waypoint = self.get_topological_node()) 
        service_response = estimates_service(request)
        return (spec, service_response)

    def _choose_new_active_batch(self, task_check_limit, now, execution_window):
        """
        Choose the tasks to execute next. 



        task_check_limit says how far along the normal task list to go to look at possible tasks
        """
        # evaluated_at_least_one_task, new_active_batch, new_active_spec, new_active_guarantees = self._choose_new_active_batch()
        
        mdp_estimates = rospy.ServiceProxy('mdp_plan_exec/get_guarantees_for_co_safe_task', GetGuaranteesForCoSafeTask)
        mdp_estimates.wait_for_service()
        last_successful_spec = None
        

        possibles = []

        # first collect the tasks which we can possibly consider
        for mdp_task in self.normal_tasks[:task_check_limit]:     

            # stop when we hit the limit 
            if len(possibles) == self.batch_limit:
                break

            # only consider tasks which we are allowed to execute from now
            if mdp_task.task.start_after < now:
                possibles.append(mdp_task)


        possibles_with_guarantees_in_time = []
        possibles_with_guarantees = []

        # now for each single task, get indpendent guarantees
        for mdp_task in possibles:

            (mdp_spec, guarantees) = self._get_guarantees_for_batch([mdp_task], estimates_service = mdp_estimates)
            # only keep tasks that are achievable on their own
            if guarantees.expected_time <= execution_window:                        
                possibles_with_guarantees_in_time.append((mdp_task, mdp_spec, guarantees))
            # keep all guarantees anyway, as we might need to report one if we can't find a task to execute
            possibles_with_guarantees.append((mdp_task, mdp_spec, guarantees))



        # sort the list of possibles by probability of success, with highest prob at start
        possibles_with_guarantees_in_time  = sorted(possibles_with_guarantees_in_time, key=lambda x: x[2].probability, reverse=True)  

        for possible in possibles_with_guarantees_in_time:
            rospy.loginfo('%s will take %.2f secs with prob %.4f' % (possible[0].action.name, possible[2].expected_time.to_sec(), possible[2].probability)) 

        # if at least one task fits into the executable time window
        if len(possibles_with_guarantees_in_time) > 0:

            # print '1'

            # keep the most probable
            new_active_batch = [possibles_with_guarantees_in_time[0][0]]
            last_successful_spec = (possibles_with_guarantees_in_time[0][1], possibles_with_guarantees_in_time[0][2])

            # print '2'
            # greedily combine with the rest
            possibles_with_guarantees_in_time = possibles_with_guarantees_in_time[1:]            

            # print '3'

            # limit the tasks inspected by the batch limit... we are skipping tasks, so just using the batch limit isn't enough
            for possible in possibles_with_guarantees_in_time:

                mdp_task = possible[0]
                # print '4'
                # stop when we hit the limit 
                if len(new_active_batch) == self.batch_limit:
                    break                

                mdp_tasks_to_check = copy(new_active_batch)
                mdp_tasks_to_check.append(mdp_task)   
                (mdp_spec, guarantees) = self._get_guarantees_for_batch(mdp_tasks_to_check, estimates_service = mdp_estimates)
               
                if guarantees.expected_time > execution_window:                        
                    rospy.loginfo('Too long policy duration for %s: %s' % (mdp_spec.ltl_task, guarantees.expected_time.to_sec())) 
                else:
                    rospy.loginfo('Acceptable policy duration for %s: %s' % (mdp_spec.ltl_task, guarantees.expected_time.to_sec()))
                    last_successful_spec = (mdp_spec, guarantees)
                    new_active_batch.append(mdp_task)

            return True, new_active_batch, last_successful_spec[0], last_successful_spec[1]

        # if we get here then at least one task can be executed now, but doesn't fit into the execution window on its own
        elif len(possibles_with_guarantees) > 0:
            return True, [], possibles_with_guarantees[0][1], possibles_with_guarantees[0][2]
        # if we get here either there are no tasks or none have passed start_after
        else:            
            return False, [], None, None





    def _next_execution_batch(self):
        """
        Called when nothing is executing and another batch of tasks are required for execution.
        """
        
        # todo: make the locking more fine-grained. Currently execution cannot be paused during this method, but the calls to the mdp services can take a long time
        with self.state_lock:
           
            now = rospy.get_rostime()

            # how late or early can a time-critical task be
            wiggle_room = rospy.Duration(180)

            self._drop_out_of_time_tasks(now, wiggle_room)

            execution_window = self.execution_window

            # now see how much time is available until the next time critical task
            if len(self.time_critical_tasks) > 0:
                next_time_critical_task = self.time_critical_tasks[0]
                until_next_critical_task = next_time_critical_task.task.execution_time - now
    
                # Don't print out all the time
                # this works well when it's looping waiting for the next tc task, but not for one-off calls
                if until_next_critical_task.secs % 20 == 0:
                    rospy.loginfo('Time until next critical task %s' % until_next_critical_task.to_sec())

                if until_next_critical_task < execution_window:
                    execution_window = until_next_critical_task


            # if we're close to a time critical task, then do that
            if len(self.time_critical_tasks) > 0 and rostime_close(now, next_time_critical_task.task.execution_time, delta = wiggle_room):                               
               
                new_active_batch = [next_time_critical_task]
                self.time_critical_tasks = SortedCollection(self.time_critical_tasks[1:], key=(lambda t: t.task.execution_time))                            
                mdp_goal = self._mdp_single_task_to_goal(next_time_critical_task)
                rospy.loginfo('executing time-critical task: %s' % mdp_goal.spec.ltl_task)
                self.mdp_exec_queue.put((mdp_goal, new_active_batch, self._get_guarantees_for_batch(new_active_batch)[1]))

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
                    
                    task_check_limit = 2 * self.batch_limit

                    evaluated_at_least_one_task, new_active_batch, new_active_spec, new_active_guarantees = self._choose_new_active_batch(task_check_limit, now, execution_window)

                    # if we found tasks to fit into the time available
                    if len(new_active_batch) > 0:
                        
                        new_normal_tasks = self.normal_tasks[task_check_limit:]

                        for mdp_task in self.normal_tasks[:task_check_limit]:
                            if mdp_task not in new_active_batch:
                                new_normal_tasks.append(mdp_task)                            

                        self.normal_tasks = SortedCollection(new_normal_tasks, key=(lambda t: t.task.end_before))                
                        
                        mdp_goal = ExecutePolicyExtendedGoal(spec = new_active_spec)
                        rospy.loginfo('Executing normal batch: %s' % mdp_goal.spec.ltl_task)
                        self.mdp_exec_queue.put((mdp_goal, new_active_batch, new_active_guarantees))
                    
                    # if we couldn't fit a batch in, but there were normal tasks available 
                    elif evaluated_at_least_one_task:
                        # if the first available task won't fit into the available execution time window, and this is the max possible, then increase the window size accordingly
                        if execution_window == self.execution_window:                        
                            # for now just increase to the expected time of last tested policy
                            self.execution_window = new_active_guarantees.expected_time 
                            rospy.loginfo('Extending default execution windown to %s' % self.execution_window.to_sec())
                        
                        # if we get here then we can't fit the first available task into the time before the first time-critical task
                        else:
                            # the basic thing here is not to recheck the normal tasks until after the next time-critical execution or until new normal tasks are added (which could be potentially earlier/shorter)
                            self.recheck_normal_tasks = False
                            # todo: we could also try some optimisation to fit in a task other than the first available normal one                    
            else:
                rospy.logdebug('No need to recheck normal tasks')
    
    def _guarantees_to_completion_time(self, guarantees):
        """
        Take a guarantees struct and determine when the execution should complete by
        """
        return rospy.get_rostime() + guarantees.expected_time + rospy.Duration(60)

    def _wait_for_policy_execution(self, expected_completion_time):
        """
        Wait until policy execution is complete or until we reach expected_completion_time at which point policy execution is preempted.
        """

        poll_time = rospy.Duration(1)
        log_count = 0
        while not self.mdp_exec_client.wait_for_result(poll_time) and not rospy.is_shutdown():
            now = rospy.get_rostime()
            remaining_secs = (expected_completion_time - now).to_sec()
            if remaining_secs < 0:
                rospy.logwarn('Policy execution did not complete in expected time, preempting')
                self.mdp_exec_client.cancel_all_goals()
                # give the policy execution some time to clean up
                complete = self.mdp_exec_client.wait_for_result(rospy.Duration(70))
                if not complete:
                    rospy.logwarn('Policy execution did not service preempt request in a reasonable time')
                    return GoalStatus.ACTIVE
                else:
                    return GoalStatus.RECALLED
                
            else:
                if log_count % 10 == 0:
                    rospy.loginfo('Another %.2f seconds until expected policy completion' % remaining_secs)
            log_count += 1

        return self.mdp_exec_client.get_state()


    def mdp_exec(self):
        """
        This is the main loop of the executor. It checks for the next goal to execute. 
        If there's nothing to execute then it calls _next_execution_batch to check for available tasks.
        """
        while not rospy.is_shutdown():

            # all encompassing try/catch to make sure this loop does not go down
            try:

                # try/catch for empty queue
                try:
                    # keep looping until paused or an Empty is thrown
                    while self.executing and not rospy.is_shutdown():

                        (mdp_goal, new_active_batch, guarantees) = self.mdp_exec_queue.get(timeout = 1)

                        sent_goal = False

                        with self.state_lock:
                            # always set active batch, but we can correct it later if we don't actually send the goal
                            self.set_active_batch(deepcopy(new_active_batch))

                            # execution status could have changed while acquiring the lock
                            if self.executing:            

                                self.mdp_exec_client = actionlib.SimpleActionClient('mdp_plan_exec/execute_policy_extended', ExecutePolicyExtendedAction)
                                self.mdp_exec_client.wait_for_server()                                
                                # last chance! -- if there was a change during wait
                                if self.executing:            
                                    self.log_task_events((m.task for m in self.active_batch), TaskEvent.TASK_STARTED, rospy.get_rostime(), description = mdp_goal.spec.ltl_task)        
                                    self.mdp_exec_client.send_goal(mdp_goal, feedback_cb = self.mdp_exec_feedback)
                                    # this is when we expect navigation to complete by
                                    expected_completion_time = self._guarantees_to_completion_time(guarantees)
                                    rospy.loginfo('Sent goal for %s' % mdp_goal.spec.ltl_task)
                                    self.republish_schedule()
                                    sent_goal = True
                                else:
                                    self.mdp_exec_client = None

                        # indicate that all processing on the task removed from the queue is complete
                        # this allows join() to work correctly
                        self.mdp_exec_queue.task_done()

                        if sent_goal:

                            final_status = self._wait_for_policy_execution(expected_completion_time)

                            with self.state_lock:
                                 
                                remaining_active = len(self.active_batch)

                                # policy execution finished everything
                                if final_status == GoalStatus.SUCCEEDED:
                                    rospy.loginfo('Policy execution succeeded')                                
                                    if remaining_active > 0:
                                        log_string = 'Active batch still contained %s task(s) after successful execution' % remaining_active
                                        rospy.loginfo(log_string)
                                        self.log_task_events((m.task for m in self.active_batch), TaskEvent.TASK_STOPPED, rospy.get_rostime(), description = log_string)        
                                        self.deactivate_active_batch()

                                # execution was paused or a task was demanded, resulting in preemption
                                elif final_status == GoalStatus.PREEMPTED:                                
                                    log_string = 'Policy execution was preempted, returning %s tasks to task lists' % remaining_active
                                    rospy.loginfo(log_string)
                                    self.log_task_events((m.task for m in self.active_batch), TaskEvent.TASK_STOPPED, rospy.get_rostime(), description = log_string)        
                                    self.deactivate_active_batch()
                                # here we may have cancelled an overrunning policy or had some other problem
                                else:             
                                    log_string = 'Policy execution exited with status %s, dropping remaining active tasks' % GoalStatus.to_string(final_status)                   
                                    rospy.loginfo(log_string)
                                    self.log_task_events((m.task for m in self.active_batch), TaskEvent.DROPPED, rospy.get_rostime(), description = log_string)        
                                    # todo: is dropping really necessary here? the tasks themselves were not aborted, just policy execution
                                    self.set_active_batch([])
                                                    
                                # make sure this can't be used now execution is complete
                                self.mdp_exec_client = None
                                # whatever happened or was executed, we should now recheck the available normal tasks
                                self.recheck_normal_tasks = True
                            self.republish_schedule()
                        else:
                            with self.state_lock:
                                rospy.loginfo('Active batch not executed, returning %s tasks to task lists' % len(self.active_batch))
                                self.deactivate_active_batch()


                except Empty, e:
                    pass

                # state of execution could have changed since the last check                
                if self.executing:            
                    self._next_execution_batch()
                else:
                    rospy.sleep(1)
            except Exception, e:
                rospy.logwarn('Caught exception in the mdp_exec loop: %s' % e)
                rospy.sleep(1)
        
        # makes publishing thread check for exit
        self.republish_schedule()


    def set_active_batch(self, batch):
        """
        Set the active batch of tasks. Also updates self.active_tasks in the base class
        """
        self.active_batch = batch
        self.active_tasks = [m.task for m in self.active_batch]

    def _process_time_critical(self):
        """
        Poll queue of new time-critical tasks and then add navigation duration to them.
        """

        while not rospy.is_shutdown():
           
            # all encompassing try/catch to make sure this loop does not go down
            try:
                 
                try:
                    mdp_task = self.new_time_critical_tasks.get(timeout = 1)

                    rospy.wait_for_service('mdp_plan_exec/get_guarantees_for_co_safe_task')
                    mdp_estimates = rospy.ServiceProxy('mdp_plan_exec/get_guarantees_for_co_safe_task', GetGuaranteesForCoSafeTask)

                    with self.state_lock:
                        rospy.loginfo('Processing new time-critical task: %s' % mdp_task.action.name)                

                        # rospy.loginfo('Time critical tasks at start of _process_time_critical:')
                        # for m in self.time_critical_tasks:
                        #     rospy.loginfo(m.task.task_id)
                        #     rospy.loginfo(m.action.name)

                        request = GetGuaranteesForCoSafeTaskRequest()
                        request.epoch = mdp_task.task.start_after
                        request.spec.ltl_task = '(F "%s")' % mdp_task.task.start_node_id
                        request.epoch = mdp_task.task.start_after
                        response = mdp_estimates(request)

                        # response gives expected time from all waypoints
                        # as we can't assume we're just doing it from where we are
                        # check from here plus the 

                        # todo: keep the full estimate around so it can be used more correctly in planning

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
                        
                        self.republish_schedule()

                        # rospy.loginfo('Time critical tasks at end of _process_time_critical:')
                        # for m in self.time_critical_tasks:
                        #     rospy.loginfo(m.task.task_id)
                        #     rospy.loginfo(m.action.name)

                    self.new_time_critical_tasks.task_done()
                except Empty, e:
                    pass
            except Exception, e:
                rospy.logwarn('Caught exception in _process_time_critical loop: %s' % e)
                rospy.sleep(1)


        

    def start_execution(self):
        """ Called when overall execution should  (re)start """
        rospy.loginfo('(Re-)starting execution')        

    def deactivate_active_batch(self):
        """
        Takes the tasks from the active batch and returns them to the approach lists for later consideration
        """
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
        self.set_active_batch([])

        return active_count

        

    def pause_execution(self):
        """ Called when overall execution should pause. This is called *before* self.executing is set to False. """

        # make sure the queue for execution is empty
        self.mdp_exec_queue.join()

        with self.state_lock:            
            self._pause_execution_internal()
           
        # wait for active batch to be empty before return
        while not rospy.is_shutdown():
            with self.state_lock:
                if self.active_batch == []:
                    return
                
            # print 'waiting for active batch to become empty'
            rospy.sleep(0.5)


    def _pause_execution_internal(self):
        """
        Does the work of pausing execution, without the lock.
        """
        # this is done by the super class *after* pause_execution completes, but we need to make sure that it is done before this lock is released to make sure execution does not continue after policy execution preemption
        self.executing = False
        
        # If the client is not None then there is execution going on. the active batch could be empty if we've just caught the tail end of execution
        # 
        # Also there could be tasks in the active batch without an action client existing. as the client is created slightly later
        if self.mdp_exec_client is not None:
            # preempt the action server
            self.mdp_exec_client.cancel_all_goals()
            rospy.loginfo('Cancelling policy execution')
        else:
            rospy.loginfo('No policy execution active when pausing')


    def task_demanded(self, demanded_task, currently_active_task):
        """ Called when a task is demanded. self.active_task is the demanded task (and is being executed) and previously_active_task was the task that was being executed (which could be None) """
       

        with self.state_lock:
            prior_execution_state = self.executing


        # this cleans up the current execution and sets self.executing to false
        self.pause_execution()            

        # todo: potential race condition -- what happens if someone calls start/pause execution here

        with self.state_lock:            
            # convert the demanded task into an mdp task for policy execution 
            demanded_mdp_task = self._convert_task_to_mdp_action(demanded_task)
            # and queue it up for execution
            mdp_goal = self._mdp_single_task_to_goal(demanded_mdp_task)
            # put blocks until the queue is empty, so we guarantee that the queue is empty while we're under lock
            tasks = [demanded_mdp_task]
            self.mdp_exec_queue.put((mdp_goal, tasks, self._get_guarantees_for_batch(tasks)[1]))
            rospy.loginfo('Queued up demanded task: %s' % (demanded_mdp_task.action.name))
            self.executing = prior_execution_state 

    def cancel_active_task(self):
        """ 
        Called to cancel the task which is currently executing.
        If something is being executed we handle this by simply pausing and restarting execution.
        pause_execution is often called before this. (this is always the case currently)
        """
        if self.executing:
            self.pause_execution()
            with self.state_lock: 
                self.executing = True

    def cancel_task(self, task_id):
        """ Called when a request is received to cancel a task. The currently executing one is checked elsewhere. """
        rospy.logwarn('Cancelling individual tasks is not yet implemented')
        return False

    def clear_schedule(self):
        """ Called to clear all tasks from schedule, with the exception of the currently executing one. """
        with self.state_lock:
            prior_execution_state = self.executing


        # this cleans up the current execution and sets self.executing to false
        self.pause_execution()            

        # (try to) make sure the queues are empty (there's a chance that between the join and next state_lock that something could be added).
        self.new_time_critical_tasks.join()
        self.mdp_exec_queue.join()

        with self.state_lock:           
            now = rospy.get_rostime() 
            self.log_task_events((m.task for m in self.normal_tasks), TaskEvent.DROPPED, now, description = 'Schedule was cleared')        
            self.normal_tasks.clear()
            self.log_task_events((m.task for m in self.time_critical_tasks), TaskEvent.DROPPED, now, description = 'Schedule was cleared')        
            self.time_critical_tasks.clear()
            self.executing = prior_execution_state 
        rospy.loginfo('All tasks cleared')


    def republish_schedule(self):
        """
        Notify schedule-publishing thread to update and publish schedule
        """
        self.update_schedule_condition.acquire()
        self.update_schedule_condition.notify()
        self.update_schedule_condition.release()

    def publish_schedule(self):
        """
        Loops continuous publishing the upcoming tasks to be executed.
        It is challenging to produce a list of the tasks that will be executed and when from this, so the compromises is that 
        ExecutionStatus contains the active batch with their execution_times set to now, all time-critical tasks and the next self.batch_limit normal tasks with their start time set to the end time of the current active batch.
        """
        while not rospy.is_shutdown():
            # all encompassing try/catch to make sure this loop does not go down
            try:

                # copy all relevant entries under lock 
                # we're taking a deepcopy as we might mess around with the times a bit
                with self.state_lock:
                    active_batch = deepcopy(self.active_batch)
                    normal_tasks = deepcopy(self.normal_tasks)
                    time_critical_tasks = deepcopy(self.time_critical_tasks)

                now = rospy.get_rostime()
                # todo: fill this value better
                expected_end_of_batch = rospy.get_rostime() + rospy.Duration(120)

                # start from the time_cr
                schedule = ExecutionStatus(currently_executing = len(active_batch) > 0)

                for m in active_batch:
                    m.task.execution_time = now
                    schedule.execution_queue.append(m.task)

                schedule.execution_queue += [m.task for m in time_critical_tasks]


                self.schedule_publisher.publish(schedule)

                self.update_schedule_condition.acquire()
                self.update_schedule_condition.wait()
                self.update_schedule_condition.release()
            except Exception, e:
                rospy.logwarn('Caught exception in publish_schedule loop: %s' % e)
                rospy.sleep(1)



if __name__ == '__main__':
    executor = MDPTaskExecutor()        
    rospy.spin()


# create a schedule class which handles blocking until execution and manages the various changes

