#!/usr/bin/env python

from __future__ import with_statement 
import rospy
from Queue import Queue, Empty
from strands_executive_msgs.msg import Task, ExecutionStatus, DurationMatrix, DurationList, ExecutePolicyAction, ExecutePolicyFeedback, ExecutePolicyGoal, MdpStateVar, StringIntPair, StringTriple, MdpAction, MdpActionOutcome, MdpDomainSpec, TaskEvent
from strands_executive_msgs.srv import GetGuaranteesForCoSafeTask, GetGuaranteesForCoSafeTaskRequest, AddCoSafeTasks, DemandCoSafeTask, GetBlacklistedNodes
from task_executor.base_executor import BaseTaskExecutor
from threading import Thread, Condition
from task_executor.execution_schedule import ExecutionSchedule
from operator import attrgetter
from math import floor
import threading
import actionlib
from task_executor.SortedCollection import SortedCollection
from task_executor.utils import rostime_to_python, rostime_close, get_start_node_ids, ros_duration_to_string, ros_time_to_string, max_duration, is_time_critical
from dateutil.tz import tzlocal
from copy import copy, deepcopy
from actionlib_msgs.msg import GoalStatus
from rosgraph_msgs.msg import Clock


ZERO = rospy.Duration(0)

class MDPTask(object):
    """
    Class to store task and mdp related stuff together.
    """

    def __init__(self, task, state_var, action, is_ltl = False, is_on_demand = False, is_interruptible = True):
        self.task = task
        self.state_var = state_var
        self.action = action
        self.is_ltl = is_ltl
        self.is_mdp_spec = False
        self.mdp_spec = None
        self.is_on_demand = is_on_demand
        self.is_interruptible = is_interruptible

    def _set_mdp_spec(self, mdp_spec):
        self.mdp_spec = mdp_spec
        self.is_mdp_spec = True
        self.is_ltl = False



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

        if rospy.get_param('use_sim_time', False):
            rospy.loginfo('Using sim time, waiting for time update')
            rospy.wait_for_message('/clock', Clock)


        # init superclasses
        super( MDPTaskExecutor, self ).__init__()

        # collection of MDPTasks sorted by deadline
        self.normal_tasks = SortedCollection(key=(lambda t: t.task.end_before))
        self.time_critical_tasks = SortedCollection(key=(lambda t: t.task.execution_time))

        # how late can tasks be expected to be before they're dropped at planning time
        self.allowable_lateness = rospy.Duration(rospy.get_param("~allowable_lateness", 300))
    
        self.state_lock = threading.Lock()
        self.mdp_exec_client = None
        self.set_active_batch([])
        self.to_cancel = set()

        # is a on-demand task active
        self.on_demand_active = False

        # only ever allow one batch in the execution queue. If this restriction is removed then demanding won't work immediately
        self.mdp_exec_queue = Queue(maxsize = 1)

        # Whether or not the normal tasks need to be checked
        self.recheck_normal_tasks = False

        # how much time should we try to fill with tasks. this is the default and will be extended if necessary
        self.execution_window = rospy.Duration(1200)
        
        # and the max number of tasks to fit into this window due to MDP scaling issues
        self.batch_limit = 5

        self.expected_completion_time = rospy.Time()
        self.mdp_exec_thread = Thread(target=self.mdp_exec)    
        self.mdp_exec_thread.start()
        
        # topic on which current schedule is broadcast
        self.schedule_publisher = rospy.Publisher('current_schedule', ExecutionStatus, latch = True, queue_size = 1)
        self.all_tasks_schedule_publisher = rospy.Publisher('task_executor/all_tasks', ExecutionStatus, latch = True, queue_size = 1)

        self.update_schedule_condition = Condition()
        self.schedule_publish_thread = Thread(target=self.publish_schedule)
        self.schedule_publish_thread.start()

        self.use_combined_sort_criteria = rospy.get_param('~combined_sort', False) 

        if self.use_combined_sort_criteria:
            rospy.loginfo('Using combined sort criteria')
        else:
            rospy.loginfo('Using separate sort criteria')

        self.advertise_services()
        self.tz = tzlocal()
        

    def add_co_safe_tasks_ros_srv(self, req):
        """
        Adds a task into the task execution framework.
        """
        try:
            self.service_lock.acquire()
            now = rospy.get_rostime()
            task_ids = []
            tasks = []
            task_spec_triples = []
            for mdp_task in req.mdp_tasks:

                task = Task()

                task.task_id = self.get_next_id()
                task_ids.append(task.task_id)
                
                task.start_after = mdp_task.start_after
                task.end_before = mdp_task.end_before
                task.priority = mdp_task.priority

                task.action = mdp_task.mdp_spec.ltl_task

                if task.start_after.secs == 0:
                    rospy.logwarn('Task %s did not have start_after set' % (task.action))                
                    task.start_after = now

                if task.end_before.secs == 0:
                    rospy.logwarn('Task %s did not have end_before set, using start_after' % (task.action))                
                    task.end_before = task.start_after

                tasks.append(task)
                task_spec_triples.append((task, mdp_task.mdp_spec, mdp_task.is_interruptible))

            self.add_specs(task_spec_triples)        
            self.log_task_events(tasks, TaskEvent.ADDED, rospy.get_rostime())                            
            return [task_ids]
        finally:    
            self.service_lock.release()
    add_co_safe_tasks_ros_srv.type=AddCoSafeTasks


    def demand_co_safe_task_ros_srv(self, req):
        """
        Demand a the task from the execution framework.
        """
        try:            
            self.service_lock.acquire()
            now = rospy.get_rostime()

            if not self.are_active_tasks_interruptible():
                return [False, 0, self.active_task_completes_by - now]

            # A task needs to be created for internal monitoring

            task = Task()
            task.task_id = self.get_next_id()
            task.start_after = req.start_after
            task.end_before = req.end_before
            task.action = req.domain_spec.ltl_task

            # give the task some sensible defaults
            if task.start_after.secs == 0:
                rospy.loginfo('Demanded task %s did not have start_after set, using now' % (task.action))                
                task.start_after = now

            if task.end_before.secs == 0:
                rospy.loginfo('Demand task %s did not have end_before set, using start_after' % (task.action))                
                # make this appear as a time-critical task
                task.end_before = now 
        
            task.execution_time = now

            # stop anything else
            if len(self.active_tasks) > 0:
                self.pause_execution()
                self.executing = False
                self.cancel_active_task()

            # and inform implementation to let it take action
            self.spec_demanded(task, req.domain_spec)                        
            
            if not self.executing:
                self.executing = True
                self.start_execution()

            self.log_task_event(task, TaskEvent.DEMANDED, rospy.get_rostime())                
            return [True, task.task_id, rospy.Duration(0)]        
        finally:    
            self.service_lock.release()

    demand_co_safe_task_ros_srv.type=DemandCoSafeTask


    def _extend_formalua_with_exec_flag(self, formula, state_var_name):
        insert_after = len(formula) - 1
        for i in range(len(formula) - 1, 0, -1):
            if formula[i] == ')':
                insert_after = i
            elif formula[i] == '(':
                break

        return formula[:insert_after] + ' & (X ' + state_var_name + '=1)' + formula[insert_after:]


    def _create_travel_mdp_task(self, waypoint):
        """ Creates an MDP task for just reacing these waypoints
            
        """
        state_var = MdpStateVar()
        action = MdpAction()                
        task = Task(action='(F "%s")' % waypoint)
        return MDPTask(task, state_var, action, is_ltl = True)


    def _convert_spec_to_mdp_action(self, task, mdp_spec, is_ltl = False, is_interruptible = True):
        """
            Converts an already formed MdpDomainSpec into our internal representation that's now a bit redundant.
        """
        mdp_task = MDPTask(task, None, None, is_ltl = is_ltl, is_interruptible = is_interruptible)
        mdp_task._set_mdp_spec(mdp_spec)
        return mdp_task


    def _convert_task_to_mdp_action(self, task):
        """ Converts a Task to a MdpAction.
            returns a task, state var, action triple
        """

        is_ltl = False
        # if this is the case then we're passed an LTL formula
        if ' ' in task.action:
            # action_name = 'n'+ str(task.task_id) + '_ltl_task'
            # state_var_name = 'executed_' + action_name

            state_var = MdpStateVar()
            outcome = MdpActionOutcome()            
            action = MdpAction()

            # task.action = self._extend_formalua_with_exec_flag(task.action, state_var_name)

            # state_var = MdpStateVar(name = state_var_name,
            #         init_val = 0, min_range = 0,
            #         max_range = 1)

            # outcome = MdpActionOutcome(probability = 1.0,
            #         post_conds = [StringIntPair(string_data = state_var_name, int_data = 1)],
            #         duration_probs = [1.0],
            #         durations = [0])
            
            # action = MdpAction(name=action_name, 
            #          pre_conds=[StringIntPair(string_data=state_var_name, int_data=0)],
            #          outcomes=[outcome])

            is_ltl = True
        else:

            action_name = 'n'+ str(task.task_id) + '_' + task.action + '_at_' + task.start_node_id.replace(' | ', '_or_')  
            # make sure there is nothing to make PRISM cry
            action_name = action_name.replace('/','_')

            state_var_name = 'executed_' + action_name

            state_var = MdpStateVar(name = state_var_name,
                    init_val = 0, min_range = 0,
                    max_range = 1)

            outcome=MdpActionOutcome(probability = 1.0,
                    post_conds = [StringIntPair(string_data = state_var_name, int_data = 1)],
                    duration_probs = [1.0],
                    durations = [task.expected_duration.to_sec()])

            action = MdpAction(name=action_name, 
                     action_server=task.action, 
                     pre_conds=[StringIntPair(string_data=state_var_name, int_data=0)],
                     outcomes=[outcome])


            if len(task.start_node_id) > 0:
                for wp in get_start_node_ids(task):
                    action.waypoints.append(wp)

            action.arguments = task.arguments

        # print state_var
        # print action

        return MDPTask(task, state_var, action, is_ltl = is_ltl)

    def add_tasks(self, tasks):
        """ Called with new tasks for the executor """

        with self.state_lock:
            
            for task in tasks:

                mdp_task = self._convert_task_to_mdp_action(task)
                if task.start_after == task.end_before:
                    self.time_critical_tasks.insert(mdp_task)
                else:
                    self.normal_tasks.insert(mdp_task)
        
        self.republish_schedule()            
        self.recheck_normal_tasks = True

    def add_specs(self, task_spec_triples):
        """ Called with new mdp_specs for the executor """

        with self.state_lock:
            
            for task, mdp_spec, is_interruptible in task_spec_triples:

                mdp_task = self._convert_spec_to_mdp_action(task, mdp_spec, is_interruptible = is_interruptible)
                if task.start_after == task.end_before:
                    self.time_critical_tasks.insert(mdp_task)
                else:
                    self.normal_tasks.insert(mdp_task)
        
        self.republish_schedule()            
        self.recheck_normal_tasks = True


    def spec_demanded(self, task, mdp_spec):
        with self.state_lock:
            prior_execution_state = self.executing

        # this cleans up the current execution and sets self.executing to false
        self.pause_execution()            

        # todo: potential race condition -- what happens if someone calls start/pause execution here

        with self.state_lock:            
            # convert the demanded task into an mdp task for policy execution 
            demanded_mdp_task = self._convert_spec_to_mdp_action(task, mdp_spec)
            demanded_mdp_task.is_on_demand = True
            # and queue it up for execution
            mdp_goal = self._mdp_single_task_to_goal(demanded_mdp_task)
            # put blocks until the queue is empty, so we guarantee that the queue is empty while we're under lock
            tasks = [demanded_mdp_task]
            self.mdp_exec_queue.put((mdp_goal, tasks, self._get_guarantees_for_batch(tasks)[1]))
            rospy.loginfo('Queued up demanded task: %s' % (demanded_mdp_task.task.action))
            self.executing = prior_execution_state 
            


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

            # print("Got Feedback: " + str(feedback))
                        
            rospy.loginfo('%s received feedback %s, %s' % (feedback.executed_action, GoalStatus.to_string(feedback.execution_status), feedback.expected_time.to_sec()))

            self.expected_completion_time = self._expected_duration_to_completion_time(feedback.expected_time)
            
            # if feedback.execution_status >= GoalStatus.PREEMPTED:
            # we don't need to check this status as we only recieve this feedback in the terminal states of the mdp, so this action is done regarless
            # todo: PREEMPTED means the action started but was cancelled during the execution if the action server
            # todo: ACTIVE means the action started but was cancelled during the execution if the action server but didn't prempt
            # todo: show we allow these to be re-added to execution queue? currently preemption signals that a task has been permanently removed
            # todo: if added back to normal tasks it will almost certainly be re-executed immediately as it's at the current location, causing a loop

            now = rospy.get_rostime()
            if feedback.executed_action != '' and self.remove_active_task(feedback.executed_action, self.goal_status_to_task_status(feedback.execution_status)):
                # update the time critical tasks based on current location
                self._update_time_critical_tasks(now)
                self.republish_schedule()

    def remove_active_task(self, action_name, task_status):
        """
        Remove the indicated task from the active batch. This is based on the (valid) assumption that the action name uniquely identifies the task.
        """
        for i in range(len(self.active_batch)):
            mdp_task = self.active_batch[i]
            if mdp_task.action is not None and mdp_task.action.name == action_name:                
                del self.active_batch[i]
                del self.active_tasks[i]
                log_string = 'Removing completed active task: %s. %s remaining in active batch' % (action_name, len(self.active_batch))
                rospy.loginfo(log_string)
                self.log_task_event(mdp_task.task, task_status, rospy.get_rostime(), description = log_string)        
                return True
        # rospy.logwarn('Could not find %s in active batch'  % action_name)
        return False

    def _check_for_late_normal_tasks(self, now):
        """ 
        Removes any normal tasks which are too late to start execution            
        """       
        dropped = False

        while len(self.normal_tasks) > 0:

            # look at the next normal task
            next_normal_task = self.normal_tasks[0]

            # drop the task if there's not enough time for expected duration to occur before the window closes
            # this ignores the navigation time for this task, making task dropping more permissive than it should be. this is ok for now.
            if now > (next_normal_task.task.end_before -  next_normal_task.task.expected_duration):
                log_string = 'Dropping queued normal task %s at %s as time window closed at %s ' % (next_normal_task.task.action, rostime_to_python(now), rostime_to_python(next_normal_task.task.end_before))
                rospy.loginfo(log_string)
                self.normal_tasks = SortedCollection(self.normal_tasks[1:], key=(lambda t: t.task.end_before))                
                self.log_task_event(next_normal_task.task, TaskEvent.DROPPED, now, description = log_string)        
                dropped = True
            else:
                break

        return dropped


    def _check_for_late_time_critical_tasks(self, now):
        """ 
        Removes any time-critical tasks which are too late to start execution            
        """
        dropped = False
        while len(self.time_critical_tasks) > 0:

            next_time_critical_task = self.time_critical_tasks[0]
            until_next_critical_task = next_time_critical_task.task.execution_time - now

            if until_next_critical_task < (ZERO - self.allowable_lateness):
                log_string = 'Dropping time-critical task %s as %s not enough time for execution' % (next_time_critical_task.action.name, until_next_critical_task.to_sec())
                rospy.loginfo(log_string)
                self.time_critical_tasks = SortedCollection(self.time_critical_tasks[1:], key=(lambda t: t.task.execution_time))
                self.log_task_event(next_time_critical_task.task, TaskEvent.DROPPED, now, description = log_string)        
                dropped = True
            else:
                break
        return dropped


    def _get_blacklisted_nodes(self):
        """
        Gets blacklisted nodes from service. If service does not exist, returns an empty list.
        """
        try:
            get_blacklisted_nodes = rospy.ServiceProxy('task_executor/get_blacklisted_nodes', GetBlacklistedNodes)
            resp = get_blacklisted_nodes()
            return resp.nodes
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return []

    def _mdp_single_task_to_goal(self, mdp_task):
        mdp_spec = self._mdp_tasks_to_spec([mdp_task])
        return ExecutePolicyGoal(spec = mdp_spec)
             

    def _mdp_tasks_to_spec(self, mdp_tasks):
        """
        Take a collection of MDPTask objects and produce an MdpDomainSpec from them.
        """
        mdp_spec = MdpDomainSpec()
        
        ltl_tasks = []
        non_ltl_tasks = []

        for mdp_task in mdp_tasks:
            if mdp_task.is_ltl:
                ltl_tasks.append(mdp_task)
            elif mdp_task.is_mdp_spec:                
                ltl_tasks.append(mdp_task)
                mdp_spec.vars.extend(mdp_task.mdp_spec.vars)
                mdp_spec.actions.extend(mdp_task.mdp_spec.actions)
            else:
                non_ltl_tasks.append(mdp_task)
                mdp_spec.vars.append(mdp_task.state_var)
                mdp_spec.actions.append(mdp_task.action)




        mdp_spec.ltl_task = ''


        task_prefix = 'F '

        # prevent the policy from visiting blacklisted nodes
        # short-term fix is to have (!X U Y) & (!X U Z), 
        # but longer term is Bruno adding G !X so we can have global invariants 
        blacklist = self._get_blacklisted_nodes()
        if len(blacklist) > 0:
            task_prefix = '(!\"%s\"' % blacklist[0]
            for bn in blacklist[1:]:            
                task_prefix += ' & !\"%s\"' % bn
            task_prefix += ') U '



        if len(non_ltl_tasks) > 0:

            for mdp_task in non_ltl_tasks:
                mdp_spec.ltl_task += '(%s %s=1) & ' % (task_prefix, mdp_task.state_var.name)
            
            mdp_spec.ltl_task = mdp_spec.ltl_task[:-3]
           # mdp_spec.ltl_task += '))'

            if len(ltl_tasks) > 0:
                mdp_spec.ltl_task += ' & '                


        if len(ltl_tasks) > 0:
            for ltl_task in ltl_tasks:
                if ltl_task.is_mdp_spec:
                    mdp_spec.ltl_task += ltl_task.mdp_spec.ltl_task
                    mdp_spec.ltl_task += ' & '
                else:
                    mdp_spec.ltl_task += ltl_task.task.action
                    mdp_spec.ltl_task += ' & '

            mdp_spec.ltl_task = mdp_spec.ltl_task[:-3]



        return mdp_spec

            

    def _drop_out_of_time_tasks(self, now):
        """
        Drop any normal or time-critical task when their time windows have been exceeded.
        """
        dropped = self._check_for_late_time_critical_tasks(now)
        dropped = dropped or self._check_for_late_normal_tasks(now)                                            


        return dropped

    def _get_guarantees_for_batch(self, task_batch, estimates_service = None, initial_waypoint = None, epoch = None):

        if epoch is None:
            epoch = rospy.get_rostime()

        if initial_waypoint is None:
            initial_waypoint = self.get_topological_node()

        if estimates_service is None:
            estimates_service = rospy.ServiceProxy('mdp_plan_exec/get_guarantees_for_co_safe_task', GetGuaranteesForCoSafeTask)
            estimates_service.wait_for_service()        
        spec = self._mdp_tasks_to_spec(task_batch)        
        request = GetGuaranteesForCoSafeTaskRequest(spec = spec, initial_waypoint = initial_waypoint, epoch = epoch)         
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
        


        possibles_with_guarantees_in_time = []
        possibles_with_guarantees = []

        # now for each single task, get indpendent guarantees
        for mdp_task in self.normal_tasks[:task_check_limit]:     

            try:
                (mdp_spec, guarantees) = self._get_guarantees_for_batch([mdp_task], estimates_service = mdp_estimates, epoch = now)
            
                # only reason about combining tasks that have their windows opena and are achievable on their own
                # 

                nav_time = max_duration(guarantees.expected_time - mdp_task.task.max_duration, ZERO)

                if False:
                    print 'timing details'
                    print ros_time_to_string(now)
                    print ros_time_to_string(mdp_task.task.start_after)
                    print ros_duration_to_string(guarantees.expected_time)
                    print ros_duration_to_string(mdp_task.task.max_duration)
                    print "Start by: %s" % ros_time_to_string(mdp_task.task.start_after - nav_time)

                if now > (mdp_task.task.start_after - nav_time):
                    if guarantees.probability > 0 and guarantees.expected_time <= execution_window:                        
                        possibles_with_guarantees_in_time.append((mdp_task, mdp_spec, guarantees))

                    # keep all guarantees anyway, as we might need to report one if we can't find a task to execute
                    possibles_with_guarantees.append((mdp_task, mdp_spec, guarantees))

            except Exception, e:
                rospy.logwarn('Ignoring task due to: %s' % e)
                self.normal_tasks.remove(mdp_task)
 


        if self.use_combined_sort_criteria:

            def task_reward(task_tuple):                
                # sanity check for zero-time case
                if task_tuple[2].expected_time.secs > 0:
                    expected_time = task_tuple[2].expected_time.to_sec()
                else:
                    expected_time = 1.0

                # sanity check for zero priority case
                if task_tuple[0].task.priority == 0:
                    rospy.logwarn('Priority is used for sorting but task %s had a priority of 0' % (task_tuple[0].task.action))
                    priority = 1.0
                else:
                    priority = task_tuple[0].task.priority

                return (priority*task_tuple[2].probability)/expected_time

            possibles_with_guarantees_in_time  = sorted(possibles_with_guarantees_in_time, key=lambda x: task_reward(x), reverse=True)    
            for possible in possibles_with_guarantees_in_time:
                rospy.loginfo('%s, with reward %.2f, will take %.2f secs with prio %s and prob %.4f ending before %s' % (possible[0].task.action, task_reward(possible), possible[2].expected_time.to_sec(), possible[0].task.priority, possible[2].probability, rostime_to_python(possible[0].task.end_before))) 

        else:

            # sort the list of possibles by probability of success, with highest prob at start
            # sort is stable, so a sequence of sorts will  work, starting with the lowest priorit

            possibles_with_guarantees_in_time  = sorted(possibles_with_guarantees_in_time, key=lambda x: x[0].task.end_before)  
            possibles_with_guarantees_in_time  = sorted(possibles_with_guarantees_in_time, key=lambda x: x[2].probability, reverse=True)  
            possibles_with_guarantees_in_time  = sorted(possibles_with_guarantees_in_time, key=lambda x: x[0].task.priority, reverse=True)  

            for possible in possibles_with_guarantees_in_time:
                rospy.loginfo('%s will take %.2f secs with prio %s and prob %.4f ending before %s' % (possible[0].task.action, possible[2].expected_time.to_sec(), possible[0].task.priority, possible[2].probability, rostime_to_python(possible[0].task.end_before))) 

        # if at least one task fits into the executable time window
        if len(possibles_with_guarantees_in_time) > 0:

            # keep the most probable
            new_active_batch = [possibles_with_guarantees_in_time[0][0]]
            last_successful_spec = (possibles_with_guarantees_in_time[0][1], possibles_with_guarantees_in_time[0][2])

            # remove the most probable from the list of possibles
            possibles_with_guarantees_in_time = possibles_with_guarantees_in_time[1:]            

            # limit the tasks inspected by the batch limit... we are skipping tasks, so just using the batch limit isn't enough
            for possible in possibles_with_guarantees_in_time:

                if len(new_active_batch) == self.batch_limit:
                    break                

                mdp_task = possible[0]
                mdp_tasks_to_check = copy(new_active_batch)
                mdp_tasks_to_check.append(mdp_task)   
                (mdp_spec, guarantees) = self._get_guarantees_for_batch(mdp_tasks_to_check, estimates_service = mdp_estimates, epoch = now)
               
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


    def _update_time_critical_tasks(self, now):
        """
        Update the execution time of each time critical task based on current location.
        """

        # todo: we don't need to always check, only when location has changed... but let's optimise later

        # how far in the future to update tasks
        only_check_in_the_next = self.execution_window * 2
        check_before = now + only_check_in_the_next

        estimates_service = rospy.ServiceProxy('mdp_plan_exec/get_guarantees_for_co_safe_task', GetGuaranteesForCoSafeTask)
        estimates_service.wait_for_service()

        new_time_critical_tasks = SortedCollection(key=(lambda t: t.task.execution_time))

        for mdp_task in self.time_critical_tasks:
            try:
                if mdp_task.task.execution_time.secs == 0 or mdp_task.task.start_after < check_before:
                    spec, guarantees = self._get_guarantees_for_batch([self._create_travel_mdp_task(mdp_task.task.start_node_id)], estimates_service = estimates_service, epoch = now)
                    # take the predicted time directly, alternative factor in the probability,
                    # see below.
                    expected_navigation_time = rospy.Duration(guarantees.expected_time.secs)
                    # prevents an underestimate due to this being the expected time to failure
                    # expected_navigation_time = rospy.Duration(guarantees.expected_time.secs /  guarantees.probability)
                    rospy.loginfo('Expected navigation time for time-critical task: %s' % expected_navigation_time.secs)    
                    mdp_task.task.execution_time = mdp_task.task.start_after - expected_navigation_time
                new_time_critical_tasks.insert(mdp_task)
            except Exception, e:
                rospy.logwarn('Dropping time-critical task due to: %s' % e)
                self.time_critical_tasks.remove(mdp_task)
                self.log_task_event(mdp_task.task, TaskEvent.DROPPED, now, description = 'Error on guarantee call. Probably due to incorrect waypoint.')        
                self.republish_schedule()

        self.time_critical_tasks = new_time_critical_tasks

        # for mdp_task in self.time_critical_tasks:
        #     print mdp_task.action.name, 'at', rostime_to_python(mdp_task.task.execution_time), 'for', rostime_to_python(mdp_task.task.start_after)


    def _should_start_next_time_critical_task(self, now):
        if len(self.time_critical_tasks) > 0:
            # if we're over the start time, it's good to go... lateness is handled in _check_for_late_time_critical_tasks
            return now > self.time_critical_tasks[0].task.execution_time
        else:
            return False

    def _next_execution_batch(self):
        """
        Called when nothing is executing and another batch of tasks are required for execution.
        """
        
        # todo: make the locking more fine-grained. Currently execution cannot be paused during this method, but the calls to the mdp services can take a long time
        with self.state_lock:
           
            now = rospy.get_rostime()

            # todo: this ignores what happens when the robot is moving, so need to check during execution too.
            self._update_time_critical_tasks(now)
        
            if self._drop_out_of_time_tasks(now):
                self.republish_schedule()

            execution_window = self.execution_window

            # now see how much time is available until the next time critical task
            if len(self.time_critical_tasks) > 0:
                next_time_critical_task = self.time_critical_tasks[0]
                until_next_critical_task = next_time_critical_task.task.execution_time - now
    
                rospy.loginfo('Time until next time-critical task: %.2f secs' % until_next_critical_task.to_sec())

                if until_next_critical_task < execution_window:
                    execution_window = until_next_critical_task

            # if we're close to a time critical task, then do that
            if self._should_start_next_time_critical_task(now):
               
                new_active_batch = [next_time_critical_task]
                self.time_critical_tasks = SortedCollection(self.time_critical_tasks[1:], key=(lambda t: t.task.execution_time))                            
                mdp_goal = self._mdp_single_task_to_goal(next_time_critical_task)
                rospy.loginfo('Executing time-critical task: %s. Start time was %s for execution at %s. Time is now %s' % (mdp_goal.spec.ltl_task, rostime_to_python(next_time_critical_task.task.execution_time), rostime_to_python(next_time_critical_task.task.start_after), rostime_to_python(now)))
                self.mdp_exec_queue.put((mdp_goal, new_active_batch, self._get_guarantees_for_batch(new_active_batch, epoch = now)[1]))

            # else see what we can squeeze into available time
            elif self.recheck_normal_tasks:

                rospy.loginfo('Checking for normal tasks to fit into available time: %.2f secs' % execution_window.to_sec())

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
                        
                        mdp_goal = ExecutePolicyGoal(spec = new_active_spec)
                        rospy.loginfo('Executing normal batch: %s' % mdp_goal.spec.ltl_task)
                        self.mdp_exec_queue.put((mdp_goal, new_active_batch, new_active_guarantees))
                    
                    # if we couldn't fit a batch in, but there were normal tasks available 
                    elif evaluated_at_least_one_task:
                        # if the first available task won't fit into the available execution time window, and this is the max possible, then increase the window size accordingly
                        if execution_window == self.execution_window and new_active_guarantees.expected_time > self.execution_window:                        
                            # for now just increase to the expected time of last tested policy
                            self.execution_window = new_active_guarantees.expected_time 
                            rospy.loginfo('Extending default execution windown to %s' % self.execution_window.to_sec())
                        
                        # if we get here then we can't fit the first available task into the time before the first time-critical task
                        else:
                            # the basic thing here is not to recheck the normal tasks until after the next time-critical execution or until new normal tasks are added (which could be potentially earlier/shorter)
                            self.recheck_normal_tasks = False
                            # todo: we could also try some optimisation to fit in a task other than the first available normal one                    
                    else:
                        # if we get here we have normal tasks, but none of them were available for execution. this probaly means
                        # that they're for the future
                        # we can't set recheck_normal_tasks to False as this is the only way the time is rechecked
                        rospy.loginfo('Next task available for execution in at most %.2f secs' % (self.normal_tasks[0].task.start_after - now).to_sec())
                        # pass
            else:
                rospy.logdebug('No need to recheck normal tasks')

            
    
    def _expected_duration_to_completion_time(self, expected_duration):
        """
        Take a guarantees struct and determine when the execution should complete by
        """

        # rospy.logwarn('expected duration ' + str(expected_duration.to_sec()))

        if expected_duration.secs < 0:
            rospy.logwarn('Expected duration was less that 0, giving a default of 5 minutes')
            expected_duration = rospy.Duration(5 * 60)

        now = rospy.get_rostime()
        expected_completion_time = now + expected_duration + rospy.Duration(60)

        # rospy.logwarn('now '  + str(rostime_to_python(now)))
        # rospy.logwarn('expected completion ' + str(rostime_to_python(expected_completion_time)))


        return expected_completion_time


    def are_active_tasks_interruptible(self):

        for mdp_task in self.active_batch:
            if not mdp_task.is_interruptible:
                return False

        return super(MDPTaskExecutor, self).are_active_tasks_interruptible()

    def _wait_for_policy_execution(self):
        """
        Wait until policy execution is complete or until we reach expected_completion_time at which point policy execution is preempted.
        """

        poll_time = rospy.Duration(5)
        overtime = rospy.Duration(0)
        # after an hour of overtime, give up
        overtime_threshold = rospy.Duration(60 * 60)

        log_count = 0

        while not self.mdp_exec_client.wait_for_result(poll_time) and not rospy.is_shutdown():


            # locking here as the feedback callback can change self.expected_completion_time
            with self.state_lock:
                now = rospy.get_rostime()
                remaining_secs = (self.expected_completion_time - now).to_sec()



            if remaining_secs < 0:

                if self.are_active_tasks_interruptible():
                    rospy.logwarn('Policy execution did not complete in expected time, preempting')
                    self.mdp_exec_client.cancel_all_goals()
                    # give the policy execution some time to clean up
                    complete = self.mdp_exec_client.wait_for_result(rospy.Duration(70))
                    if not complete:
                        rospy.logwarn('Policy execution did not service preempt request in a reasonable time')
                        return GoalStatus.ACTIVE
                    else:
                        return GoalStatus.PREEMPTED
                else:
                    rospy.logwarn('Policy execution did not complete in expected time, but is non-interruptible, so waiting. Overtime: %ss' % ros_duration_to_string(overtime))
                    overtime += poll_time

                if overtime > overtime_threshold:
                    rospy.logwarn('Policy execution has exceeded overtime threshold all execution flags ignored, preempting regardless')
                    self.mdp_exec_client.cancel_all_goals()
                    # give the policy execution some time to clean up
                    complete = self.mdp_exec_client.wait_for_result(rospy.Duration(70))
                    if not complete:
                        rospy.logwarn('Policy execution did not service preempt request in a reasonable time')
                        return GoalStatus.ACTIVE
                    else:
                        return GoalStatus.RECALLED

            else:
                if log_count % 3 == 0:
                    rospy.loginfo('Another %.2f seconds until expected policy completion' % remaining_secs)
            log_count += 1

            with self.state_lock:
                # check whether we're due to start a time-critical task that we'd otherwise miss
                if self._should_start_next_time_critical_task(now):
                    if self.on_demand_active:                                                    
                        rospy.logwarn('Ignoring the start of a time-critical task due to an on-demand task')
                    else:
                        rospy.logwarn('We should be executing a time-critical task now, so cancelling execution')
                        self.mdp_exec_client.cancel_all_goals()
                        complete = self.mdp_exec_client.wait_for_result(rospy.Duration(70))
                        if not complete:
                            rospy.logwarn('Policy execution did not service preempt request in a reasonable time')
                            return GoalStatus.ACTIVE
                        else:
                            return GoalStatus.PREEMPTED

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

                            self.republish_schedule()

                            # execution status could have changed while acquiring the lock
                            if self.executing:            

                                self.mdp_exec_client = actionlib.SimpleActionClient('mdp_plan_exec/execute_policy', ExecutePolicyAction)
                                self.mdp_exec_client.wait_for_server()                                
                                # last chance! -- if there was a change during wait
                                if self.executing:            
                                    self.log_task_events((m.task for m in self.active_batch), TaskEvent.TASK_STARTED, rospy.get_rostime(), description = mdp_goal.spec.ltl_task)        
                                    self.mdp_exec_client.send_goal(mdp_goal, feedback_cb = self.mdp_exec_feedback)
                                    # this is when we expect navigation to complete by
                                    self.expected_completion_time = self._expected_duration_to_completion_time(guarantees.expected_time)
                                    rospy.loginfo('Sent goal for %s' % mdp_goal.spec.ltl_task)
                                    self.republish_schedule()

                                    for m in self.active_batch:
                                        self.on_demand_active = self.on_demand_active or m.is_on_demand

                                    if self.on_demand_active:
                                        rospy.loginfo('This is an on-demand task')

                                    sent_goal = True
                                else:
                                    self.mdp_exec_client = None

                        # indicate that all processing on the task removed from the queue is complete
                        # this allows join() to work correctly
                        self.mdp_exec_queue.task_done()

                        if sent_goal:


                            final_status = self._wait_for_policy_execution()

                            with self.state_lock:
                                 
                                # these are left after execution
                                

                                # remove those tasks which were part of the cancelled set
                                # print self.to_cancel
                                

                                active_tasks = []
                                cancelled_tasks = []
                                for m in self.active_batch:

                                    # print m.task.task_id 

                                    if m.task.task_id in self.to_cancel:
                                        # print 'cancelled'
                                        cancelled_tasks.append(m)
                                    else:
                                        # print 'active'
                                        active_tasks.append(m)

                                self.active_batch = active_tasks
                                self.to_cancel = []

                                # print cancelled_tasks
                                # print self.active_batch

                                if len(cancelled_tasks) > 0:
                                    log_string = 'Dropped %s task(s) after execution due to cancellation' % len(cancelled_tasks)
                                    rospy.loginfo(log_string)
                                    self.log_task_events((m.task for m in cancelled_tasks), TaskEvent.DROPPED, rospy.get_rostime(), description = log_string)        
                                
                                remaining_active = len(self.active_batch)
                                self.on_demand_active = False                                

                                # policy execution finished everything
                                #if final_status == GoalStatus.SUCCEEDED or final_status == GoalStatus.PREEMPTED:
                                if True: #This way tasks arent dropped when navigation failures occur. TODO see whether the stuff under the else statement is needed for some cases.                                
                                    self.deactivate_active_batch(goal_status = final_status)
                                
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
                            
                        else:
                            with self.state_lock:
                                self.deactivate_active_batch(goal_status = GoalStatus.RECALLED, save_all = True)

                        self.republish_schedule()

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
        self.active_batch = copy(batch)
        self.active_tasks = [m.task for m in self.active_batch]
                  

    def start_execution(self):
        """ Called when overall execution should  (re)start """
        rospy.loginfo('(Re-)starting execution')        

    def deactivate_active_batch(self, goal_status, save_all = False, description = ''):
        """
        Takes the tasks from the active batch and returns them to the approach lists for later consideration.  
        """
        active_count = len(self.active_batch)
        now = rospy.get_rostime()

        log_string = 'De-activating remaining %s tasks after execution finished with status %s.' % (active_count, GoalStatus.to_string(goal_status))

        if active_count > 0:
            
            if save_all:
                log_string += ' Saving all back to task list.'

                for mdp_task in self.active_batch:                    
                    if mdp_task.task.start_after == mdp_task.task.end_before:                        
                        self.time_critical_tasks.insert(mdp_task)
                    else:
                        self.normal_tasks.insert(mdp_task)                                                

            else:
                # for each task remaining in the active batch, put it back into the right list

                do_not_reactivate_later = []
                reactivate_later = []
                for mdp_task in self.active_batch:                    
                    # we can't monitor the execution of these tasks, so we always assume they're done when deactivated
                    if mdp_task.is_ltl or mdp_task.is_mdp_spec or mdp_task.is_on_demand:
                        do_not_reactivate_later.append(mdp_task)
                    else:
                        reactivate_later.append(mdp_task)
                
                self.log_task_events((m.task for m in do_not_reactivate_later), self.goal_status_to_task_status(goal_status), now, description = log_string + ' Cannot be reactivated later.')                                                 
                self.log_task_events((m.task for m in reactivate_later), TaskEvent.TASK_STOPPED, now, description = log_string + ' Saved task to reactivate later')             
                    
                for mdp_task in reactivate_later:                    
                    if mdp_task.task.start_after == mdp_task.task.end_before:                        
                        self.time_critical_tasks.insert(mdp_task)
                    else:
                        self.normal_tasks.insert(mdp_task)                                                

            # empty the active batch. this might mean some feedback misses the update
            # the consequence is that the task was completed but we preempted before receiving the update, 
            # this means the task will be executed again, but there's no easy way around this
            self.set_active_batch([])

        rospy.loginfo(log_string)
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
            demanded_mdp_task.is_on_demand = True
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
            
            # save the current executing tasks to drop later
            with self.state_lock:
                self.to_cancel = set([m.task.task_id for m in self.active_batch])

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
        self.mdp_exec_queue.join()

        with self.state_lock:           
            now = rospy.get_rostime() 
            self.log_task_events((m.task for m in self.normal_tasks), TaskEvent.DROPPED, now, description = 'Schedule was cleared')        
            self.normal_tasks.clear()
            self.log_task_events((m.task for m in self.time_critical_tasks), TaskEvent.DROPPED, now, description = 'Schedule was cleared')        
            self.time_critical_tasks.clear()
            self.executing = prior_execution_state 
        
        self.republish_schedule()                        
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
                    expected_completion_time = deepcopy(self.expected_completion_time)
                    active_batch = deepcopy(self.active_batch)
                    normal_tasks = deepcopy(self.normal_tasks)
                    time_critical_tasks = deepcopy(self.time_critical_tasks)

                now = rospy.get_rostime()
                # todo: fill this value better
                expected_end_of_batch = rospy.get_rostime() + rospy.Duration(120)

                # start from the time_cr
                schedule = ExecutionStatus(currently_executing = len(active_batch) > 0)
                all_tasks = ExecutionStatus(currently_executing = len(active_batch) > 0)

                schedule.header.stamp = now
                all_tasks.header.stamp = now

                for m in active_batch:
                    m.task.execution_time = now
                    schedule.execution_queue.append(m.task)
                    all_tasks.execution_queue.append(m.task)


                # schedule.execution_queue += [m.task for m in time_critical_tasks]
                all_tasks.execution_queue += [m.task for m in time_critical_tasks]

                all_tasks.execution_queue += [m.task for m in normal_tasks]        
                all_tasks.execution_queue = sorted(all_tasks.execution_queue, key=lambda x: x.start_after)  
                all_tasks.execution_queue = sorted(all_tasks.execution_queue, key=lambda x: x.priority)  


                self.schedule_publisher.publish(schedule)    
                self.all_tasks_schedule_publisher.publish(all_tasks)

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

